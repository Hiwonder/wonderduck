# TF 卡镜像备份

关闭机器人，将 TF 卡插入 Linux 电脑，在保存镜像的目录执行：

```bash
sudo python3 /path/to/wonderduck/scripts/backup-tf-image.py
```

请将 `/path/to/wonderduck` 替换为仓库的实际路径。脚本会生成可直接烧录的 `wonderduck_rk3566_YYYYMMDD.img`、压缩副本 `.img.gz` 及各自的校验文件；烧录后首次启动自动扩容。压缩副本会验证与原始镜像一致，压缩失败时原始镜像仍会保留。

备份镜像会清除私人 Wi-Fi、蓝牙配对、SSH 授权、交互历史和历史录制，并在新机首次启动时生成独立的 SSH 主机密钥和机器标识。热点密码为 `hiwonder`，登录密码、当前和默认模型、动作组和校准数据会保留。镜像中仍可能含有其他用户数据，分享前请检查。
