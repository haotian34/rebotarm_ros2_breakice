"""Local safety buttons. Software control requires a working ROS/CAN connection."""
import tkinter as tk

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def main(args=None):
    rclpy.init(args=args)
    node = Node('rebotarm_safety_panel')
    node.declare_parameter('arm_namespace', 'rebotarm')
    namespace = str(node.get_parameter('arm_namespace').value).strip('/')
    clients = {name: node.create_client(Trigger, f'/{namespace}/{name}')
               for name in ('safety_stop', 'stop_and_home')}
    root = tk.Tk()
    root.title('机械臂停止与缓慢复位')
    status = tk.StringVar(value='回零前确保路径无人；软件按钮不能替代硬件急停。')
    pending = []

    def send(name):
        client = clients[name]
        if not client.service_is_ready():
            status.set('控制器服务不可用，请使用硬件急停！')
            return
        pending.append(client.call_async(Trigger.Request()))
        status.set('正在请求停止…')

    tk.Button(root, text='停止并缓慢复位', bg='#b71c1c', fg='white',
              font=('', 24), command=lambda: send('stop_and_home')).pack(fill='x', padx=16, pady=16)
    tk.Button(root, text='仅停止 / 中断复位（空格键）', font=('', 18),
              command=lambda: send('safety_stop')).pack(fill='x', padx=16, pady=8)
    tk.Label(root, textvariable=status, wraplength=540).pack(padx=16, pady=16)
    root.bind('<space>', lambda event: send('safety_stop'))

    def poll():
        rclpy.spin_once(node, timeout_sec=0)
        for future in pending[:]:
            if future.done():
                try:
                    response = future.result()
                    status.set(('已受理：' if response.success else '失败：') + response.message)
                except Exception as exc:
                    status.set(f'通信失败：{exc}；请使用硬件急停')
                pending.remove(future)
        root.after(20, poll)

    root.after(20, poll)
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()
