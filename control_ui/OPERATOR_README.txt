操作员一键启动说明

1. 双击 run_ui.bat。
2. 等待浏览器自动打开 H5 显控页面。
3. 启动窗口里出现 System is ready 表示机械臂节点已在线。
4. 如果窗口显示 H5 is open, but arm nodes are not fully online，先不要操作机械臂，联系工程人员检查 Jetson 节点。
5. 如果窗口显示 FAIL，不要关闭窗口，把窗口内容拍照发给工程人员。

工程人员手动 SSH 测试命令：

ssh -o BatchMode=yes -o ConnectTimeout=5 mumu@192.168.1.35 "echo SSH_OK"

