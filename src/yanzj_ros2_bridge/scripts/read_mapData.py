#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, json, math
from collections import Counter
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rtabmap_msgs.msg import MapData
from cv_bridge import CvBridge
from IPython import embed
from rvl_decoder import decompress_rvl_depth, is_rvl_format


def quat_to_mat(qx, qy, qz, qw):
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw) or 1.0
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz
    return np.array([
        [1-2*(yy+zz), 2*(xy-wz),   2*(xz+wy)],
        [2*(xy+wz),   1-2*(xx+zz), 2*(yz-wx)],
        [2*(xz-wy),   2*(yz+wx),   1-2*(xx+yy)]
    ], dtype=np.float64)

def pose_to_dict(p):
    return {
        "tx": float(p.position.x), "ty": float(p.position.y), "tz": float(p.position.z),
        "qx": float(p.orientation.x), "qy": float(p.orientation.y),
        "qz": float(p.orientation.z), "qw": float(p.orientation.w)
    }

def transform_to_dict(tf):
    return {
        "tx": float(tf.translation.x), "ty": float(tf.translation.y), "tz": float(tf.translation.z),
        "qx": float(tf.rotation.x), "qy": float(tf.rotation.y),
        "qz": float(tf.rotation.z), "qw": float(tf.rotation.w)
    }

def camera_info_to_dict(ci):
    if ci is None:
        return None
    return {
        "width": int(ci.width), "height": int(ci.height),
        "distortion_model": ci.distortion_model,
        "D": [float(x) for x in ci.d],
        "K": [float(x) for x in ci.k],
        "R": [float(x) for x in ci.r],
        "P": [float(x) for x in ci.p],
        "binning_x": int(ci.binning_x), "binning_y": int(ci.binning_y),
    }

class MapDataExtractor(Node):
    def __init__(self):
        super().__init__('mapdata_extractor_nodes_only')

        self.declare_parameter('topic', '/rtabmap_ranger_xarm/mapData')
        self.declare_parameter('out_dir', './rtabmap_dump')

        self.loop_closure_types = {1, 2, 3, 4} 

        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.out_dir = os.path.abspath(self.get_parameter('out_dir').get_parameter_value().string_value)

        os.makedirs(self.out_dir, exist_ok=True)
        os.makedirs(os.path.join(self.out_dir, "rgb"), exist_ok=True)
        os.makedirs(os.path.join(self.out_dir, "depth"), exist_ok=True)
        os.makedirs(os.path.join(self.out_dir, "camera_info"), exist_ok=True)

        self.bridge = CvBridge()
        self.msg_count = 0

        # 缓存
        self.frames = {}        # id -> {"stamp":(sec,nsec), "rgb":path, "depth":path}
        self.poses_node = {}    # id -> pose dict (map->node)
        self.local_tf = {}      # id -> transform dict (node->camera), 可选
        self.prev_id2pose = {}

        # 回环检测阈值
        self.trans_thresh = 0.05
        self.rot_thresh = math.radians(1.0)

        self.sub = self.create_subscription(MapData, self.topic, self.cb, 10)
        self.get_logger().info(f"Listening: {self.topic}, output -> {self.out_dir}")

    def cb(self, msg: MapData):
        self.msg_count += 1

        # 1) id->优化后位姿（map->node）
        id2pose = {nid: p for nid, p in zip(msg.graph.poses_id, msg.graph.poses)}

        # 2) 保存新增帧载荷（RGB/Depth/CameraInfo/LocalTF）—只保存一次
        for n in msg.nodes:
            if n.id not in self.frames:
                self._save_node_payload_once(n)

        # 3) 覆盖更新节点位姿（map->node）
        for nid, p in id2pose.items():
            self.poses_node[nid] = pose_to_dict(p)

        # 4) 回环检测
        links_changed, added_links = self._check_links_changed(msg.graph.links)
        pose_changed, affected_nodes = self._check_pose_jump(self.prev_id2pose, id2pose)

        self.prev_id2pose = id2pose

        if links_changed or pose_changed:
            # 合并受影响节点：位姿跳变的 + 新增回环边端点
            affected_ids = set(affected_nodes)
            for a, b, _t in added_links:
                affected_ids.add(a)
                affected_ids.add(b)
            affected_ids = sorted(affected_ids)

            extra = {
                "loop_links_added": added_links,  # [(a,b,type), ...]
                "pose_jump_detected": bool(pose_changed),
                "links_changed": bool(links_changed),
            }

            self._flush(reason="loop_detected", affected_nodes=affected_ids, extra=extra)
        
        else:
            self._flush(reason="periodic")


    def _save_node_payload_once(self, n):
        nid = n.id
        stamp = (n.data.header.stamp.sec, n.data.header.stamp.nanosec)

        # RGB
        rgb = None
        if n.data.left.data and n.data.left.encoding:
            try:
                rgb = self.bridge.imgmsg_to_cv2(n.data.left, desired_encoding='bgr8')
            except Exception:
                rgb = None
        if rgb is None and n.data.left_compressed:
            arr = np.frombuffer(bytes(n.data.left_compressed), dtype=np.uint8)
            rgb = cv2.imdecode(arr, cv2.IMREAD_COLOR)

        # Depth
        depth = None
        depth_path = None
        if n.data.right.data and n.data.right.encoding:
            depth = self.bridge.imgmsg_to_cv2(n.data.right)
        elif n.data.right_compressed:
            arr = np.frombuffer(bytes(n.data.right_compressed), dtype=np.uint8)
            depth = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)

        # 保存
        rgb_path = None
        if rgb is not None:
            rgb_path = os.path.join(self.out_dir, "rgb", f"{nid}.png")
            cv2.imwrite(rgb_path, rgb)
        
        if depth is not None:
            # 深度图统一保存为.npy格式以保持16位精度
            depth_path = os.path.join(self.out_dir, "depth", f"{nid}.npy")
            np.save(depth_path, depth)
            
            # 记录详细信息
            if depth.dtype == np.uint16:
                self.get_logger().info(f"[id={nid}] Saved 16-bit depth: {depth.shape}, range={depth.min()}-{depth.max()}mm")
            elif depth.dtype == np.uint8:
                self.get_logger().warning(f"[id={nid}] Saved 8-bit depth (precision lost): {depth.shape}, range={depth.min()}-{depth.max()}")

        # CameraInfo（可选）
        left_ci = n.data.left_camera_info[0] if len(n.data.left_camera_info) > 0 else None
        right_ci = n.data.right_camera_info[0] if len(n.data.right_camera_info) > 0 else None
        ci_rec = {"left": camera_info_to_dict(left_ci), "right": camera_info_to_dict(right_ci)}
        with open(os.path.join(self.out_dir, "camera_info", f"{nid}.json"), "w") as f:
            json.dump(ci_rec, f, indent=2)

        # Local TF（node->camera），仅保存不使用
        if len(n.data.local_transform) > 0:
            self.local_tf[nid] = transform_to_dict(n.data.local_transform[0])

        self.frames[nid] = {"stamp": stamp, "rgb": rgb_path, "depth": depth_path}
        self.get_logger().info(f"[id={nid}] saved payload: rgb={bool(rgb_path)}, depth={bool(depth_path)}, cam_info={'Y' if ci_rec['left'] or ci_rec['right'] else 'N'}")


    def _check_links_changed(self, links):
        """检查是否有新的回环边被添加"""
        # 获取当前所有边
        current_links = {(l.from_id, l.to_id, l.type) for l in links}
        
        # 获取之前的边集合
        prev_links = getattr(self, 'prev_links_set', set())
        
        # 计算新增的边
        added_links = current_links - prev_links
        
        # 更新历史
        self.prev_links_set = current_links
        
        is_loop = False
        
        # # link type
        # enum Type {
        #     kNeighbor,           // 0: 相邻节点（里程计边）
        #     kGlobalClosure,      // 1: 全局回环
        #     kLocalSpaceClosure,  // 2: 局部空间回环
        #     kLocalTimeClosure,   // 3: 局部时间回环
        #     kUserClosure,        // 4: 用户定义回环
        #     kVirtualClosure,     // 5: 虚拟回环
        #     kNeighborMerged,     // 6: 合并的相邻节点
        #     kPosePrior,          // 7: 绝对位姿先验
        #     kLandmark,           // 8: 地标
        #     kGravity,            // 9: 重力约束
        #     kEnd
        # };

        # 检查新增的边中是否有回环边
        # 根据rtabmap的Link类型定义：
        # 0: kNeighbor (里程计边)
        # 1: kGlobalClosure (全局回环)
        # 2: kLocalSpaceClosure (局部空间回环) 
        # 3: kLocalTimeClosure (局部时间回环)
        # 4: kUserClosure (用户定义回环)
        # 5: kVirtualClosure (虚拟回环)
        # 6: kNeighborMerged (合并的相邻节点)
    
        if added_links:
            for from_id, to_id, link_type in added_links:
                if link_type in self.loop_closure_types:
                    is_loop = True
                    self.get_logger().info(f"Loop closure detected: {from_id} -> {to_id}, type={link_type}")
                
        return is_loop, added_links


    def _yaw_from_quat(self, x, y, z, w):
        siny_cosp = 2*(w*z + x*y)
        cosy_cosp = 1 - 2*(y*y + z*z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _check_pose_jump(self, prev, curr):
        """检查位姿跳跃，返回受影响的节点列表"""
        affected_nodes = []
        for nid, p in curr.items():
            if nid not in prev:
                continue
            a, b = prev[nid], p
            dt = math.sqrt((a.position.x-b.position.x)**2 + (a.position.y-b.position.y)**2 + (a.position.z-b.position.z)**2)
            ya = self._yaw_from_quat(a.orientation.x, a.orientation.y, a.orientation.z, a.orientation.w)
            yb = self._yaw_from_quat(b.orientation.x, b.orientation.y, b.orientation.z, b.orientation.w)
            dy = abs((yb - ya + math.pi) % (2*math.pi) - math.pi)
            if dt > self.trans_thresh or dy > self.rot_thresh:
                affected_nodes.append(nid)

        return len(affected_nodes) > 0, affected_nodes



    def _flush(self, reason="periodic", affected_nodes=None, extra=None):
        # 位姿（map->node）——覆盖写，作为当前快照
        with open(os.path.join(self.out_dir, "poses_nodes.json"), "w") as f:
            json.dump(self.poses_node, f, indent=2)

        # 可选保存 local TF（node->camera）——覆盖写
        if self.local_tf:
            with open(os.path.join(self.out_dir, "local_transforms.json"), "w") as f:
                json.dump(self.local_tf, f, indent=2)

        # 若是回环/跳变触发，把事件记录到日志（追加）
        if affected_nodes is not None:
            rec = {
                "msg_index": self.msg_count,
                "reason": reason,
                "affected_nodes": affected_nodes,          # [id, id, ...]
                "total_nodes": len(self.poses_node),
                "extra": (extra or {})                     # {"loop_links_added":[(a,b,t),...], ...}
            }
            events_path = os.path.join(self.out_dir, "events.jsonl")
            with open(events_path, "a") as f:
                f.write(json.dumps(rec) + "\n")

            self.get_logger().info(
                f"poses_nodes.json updated ({reason}). count={len(self.poses_node)}, "
                f"affected={len(affected_nodes)} nodes"
            )
        else:
            self.get_logger().info(
                f"poses_nodes.json updated ({reason}). count={len(self.poses_node)}"
            )


    # def _flush(self, reason="periodic", affected_nodes=None):
    #     # 位姿（map->node）
    #     with open(os.path.join(self.out_dir, "poses_nodes.json"), "w") as f:
    #         json.dump(self.poses_node, f, indent=2)
        
    #     # 可选保存 local TF（node->camera）
    #     if self.local_tf:
    #         with open(os.path.join(self.out_dir, "local_transforms.json"), "w") as f:
    #             json.dump(self.local_tf, f, indent=2)
        
    #     # 保存受影响节点信息（用于语义重建）
    #     if affected_nodes:
    #         affected_info = {
    #             "timestamp": self.msg_count,
    #             "reason": reason,
    #             "affected_nodes": affected_nodes,
    #             "total_nodes": len(self.poses_node)
    #         }
    #         with open(os.path.join(self.out_dir, "affected_nodes.json"), "w") as f:
    #             json.dump(affected_info, f, indent=2)
    #         self.get_logger().info(f"poses_nodes.json updated ({reason}). count={len(self.poses_node)}, affected={len(affected_nodes)} nodes")
    #     else:
    #         self.get_logger().info(f"poses_nodes.json updated ({reason}). count={len(self.poses_node)}")


def main():
    rclpy.init()
    node = MapDataExtractor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
