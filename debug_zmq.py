#!/usr/bin/env python3

import zmq
import time
import sys

def test_port(ip, port, timeout=5):
    """测试指定端口是否能接收到数据"""
    print(f"测试端口 {port}...")
    
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.SUBSCRIBE, b'')
    socket.setsockopt(zmq.RCVTIMEO, timeout * 1000)  # 设置超时时间
    
    try:
        socket.connect(f"tcp://{ip}:{port}")
        print(f"  连接到 tcp://{ip}:{port}")
        
        # 尝试接收数据
        data = socket.recv_pyobj()
        print(f"  ✅ 成功接收到数据: {type(data)}")
        if isinstance(data, dict):
            print(f"     数据键: {list(data.keys())}")
        return True
        
    except zmq.error.Again:
        print(f"  ❌ 超时 - 没有接收到数据")
        return False
    except Exception as e:
        print(f"  ❌ 错误: {e}")
        return False
    finally:
        socket.close()
        context.term()

def main():
    ip = "10.29.228.185"
    
    print("开始测试ZMQ连接...")
    print(f"目标IP: {ip}")
    print()
    
    # 测试所有端口
    ports = [4401, 4402, 4403, 4404]
    results = {}
    
    for port in ports:
        results[port] = test_port(ip, port)
        print()
    
    # 总结
    print("测试结果总结:")
    for port, success in results.items():
        status = "✅ 成功" if success else "❌ 失败"
        print(f"  端口 {port}: {status}")
    
    # 检查是否所有必需的端口都工作
    required_ports = [4401, 4403, 4404]  # 客户端需要的端口
    working_ports = [port for port in required_ports if results[port]]
    
    if len(working_ports) == len(required_ports):
        print("\n🎉 所有必需的端口都在工作！")
    else:
        print(f"\n⚠️  只有 {len(working_ports)}/{len(required_ports)} 个必需端口在工作")
        print(f"   工作的端口: {working_ports}")
        print(f"   失败的端口: {[p for p in required_ports if p not in working_ports]}")

if __name__ == "__main__":
    main() 