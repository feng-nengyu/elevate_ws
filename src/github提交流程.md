# `elevate_ws/src` Git 管理与项目交接指南

本文用于管理 `/home/xie/wzl/elevate_ws/src` 中的源代码，并说明如何把项目完整交接给下一位开发者。

## 1. 当前项目说明

当前 Git 仓库只管理 ROS 2 工作区的 `src` 目录：

```text
elevate_ws/                 # ROS 2 工作区，不在当前 Git 仓库中
├── src/                    # Git 仓库根目录
│   ├── piper/
│   ├── piper_launch/
│   ├── piper_vision/
│   ├── OrbbecSDK_ROS2/     # Git 子模块
│   ├── .gitignore
│   └── ...
├── build/                  # 编译产物，不提交
├── install/                # 编译产物，不提交
└── log/                    # 编译日志，不提交
```

GitHub 仓库：

```text
https://github.com/xingoinglu/robotics_arms_elevate
```

远程仓库别名为 `origin`，主分支为 `main`。

## 2. Git 管理的基本关系

```text
工作区中的文件
      │ git add
      ▼
暂存区
      │ git commit
      ▼
本地 Git 仓库
      │ git push
      ▼
GitHub 远程仓库
```

- `git status`：查看哪些文件发生了变化。
- `git diff`：查看尚未暂存的具体修改。
- `git add`：选择要放入下一次提交的修改。
- `git diff --cached`：检查已经暂存、准备提交的修改。
- `git commit`：在本地生成一个版本记录。
- `git pull --rebase`：获取 GitHub 的更新，并把本地提交接到最新版本后面。
- `git push`：把本地提交上传到 GitHub。

## 3. 日常代码管理流程

### 3.1 开始工作前同步代码

进入源代码仓库：

```bash
cd /home/xie/wzl/elevate_ws/src
```

确认当前分支和文件状态：

```bash
git status
git branch --show-current
```

同步 GitHub 上的最新代码：

```bash
git pull --rebase origin main
git submodule update --init --recursive
```

如果有尚未提交的本地修改，`git pull --rebase` 可能会拒绝执行。此时应先提交修改，或者使用 `git stash` 临时保存。

### 3.2 修改代码后检查内容

```bash
git status
git diff
```

不要只看文件名，还应使用 `git diff` 确认每一处修改都是需要提交的内容。

### 3.3 添加本次需要提交的文件

推荐明确添加文件：

```bash
git add piper_launch/launch/all.launch.py
git add piper_vision/piper_vision/example.py
```

不建议不检查就直接执行 `git add .`，因为它可能把无关修改一起加入提交。

检查暂存区：

```bash
git status
git diff --cached
```

如果文件加错了，可以取消暂存，同时保留本地修改：

```bash
git restore --staged 路径/文件名
```

### 3.4 提交代码

```bash
git commit -m "类型: 简要说明修改内容"
```

常用提交类型：

```text
feat: 新增功能
fix: 修复问题
docs: 修改文档
refactor: 重构代码，但不改变功能
test: 添加或修改测试
chore: 修改构建、配置或其他维护内容
```

示例：

```bash
git commit -m "fix: 修复机械臂轨迹执行注释"
```

一次提交尽量只处理一个明确问题，避免把多个不相关的修改放在一起。

### 3.5 上传到 GitHub

提交后先同步远程代码：

```bash
git pull --rebase origin main
```

没有冲突时上传：

```bash
git push origin main
```

看到类似下面的结果表示上传成功：

```text
main -> main
```

最后检查：

```bash
git status
```

正常状态应包含：

```text
Your branch is up to date with 'origin/main'.
nothing to commit, working tree clean
```

## 4. 不应提交到 GitHub 的内容

当前 `.gitignore` 已排除大部分生成文件和敏感文件，包括：

- `build/`、`install/`、`log/`；
- Python 缓存和虚拟环境；
- `.env`、私钥和本机配置；
- ROS bag、相机录像和运行时图片；
- `.pt`、`.onnx`、`.engine` 等模型或大文件。

提交前检查：

```bash
git status
git diff --cached
```

禁止提交：

- GitHub Token、密码、SSH 私钥；
- 设备账号和内部网络信息；
- 未脱敏的个人数据；
- 体积很大的模型、数据集和录像；
- 只适用于当前电脑的绝对路径配置。

需要交接但不能上传 GitHub 的文件，应通过安全的共享方式单独传递，并在交接记录中写清文件名称、用途和放置位置。

## 5. 子模块管理

本项目使用 `OrbbecSDK_ROS2` 子模块：

```text
https://github.com/orbbec/OrbbecSDK_ROS2.git
```

主仓库只保存子模块的地址和指定提交，不直接保存子模块的全部提交历史。

初始化或恢复子模块：

```bash
cd /home/xie/wzl/elevate_ws/src
git submodule update --init --recursive
```

查看子模块状态：

```bash
git submodule status
```

如果只是使用第三方 SDK，不要随意在子模块目录中修改和提交代码。确实需要修改时，应先确认子模块仓库的提交权限和维护方式。

## 6. 交接前：当前开发者要做什么

### 6.1 整理和检查代码

```bash
cd /home/xie/wzl/elevate_ws/src
git status
git diff
```

删除不需要的临时文件，确认敏感信息和大文件没有被 Git 跟踪。

检查已经被 Git 跟踪的文件：

```bash
git ls-files
```

注意：把文件加入 `.gitignore` 并不能自动删除以前已经提交的内容。如果敏感信息曾经进入 Git 历史，应先撤销相关凭证，并单独清理历史记录。

### 6.2 提交并上传最后修改

```bash
git add 需要交接的文件
git diff --cached
git commit -m "docs: 完善项目交接说明"
git pull --rebase origin main
git push origin main
```

如果没有新修改，`git commit` 会提示没有内容可提交，可以直接继续检查同步状态。

### 6.3 记录最终版本

获取交接版本信息：

```bash
git branch --show-current
git rev-parse HEAD
git log -1 --oneline
git remote -v
git submodule status
```

把以下内容发给接手人：

- GitHub 仓库地址；
- 使用的分支名称，通常为 `main`；
- 最终提交编号；
- ROS 2 和 Ubuntu 版本；
- 设备型号、连接方式和驱动要求；
- 编译、启动和测试命令；
- 未完成事项和已知问题；
- 未上传 GitHub 的模型、标定和本机配置如何获取。

### 6.4 完成交接前最终确认

```bash
git fetch origin
git status
git log --oneline --decorate -5
```

应确认本地 `main` 与 `origin/main` 一致，并到 GitHub 网页检查最终提交确实存在。

## 7. 接手人：下载完整项目

### 7.1 安装基础工具

接手人的电脑需要安装：

- Ubuntu 和 ROS 2 Humble；
- Git；
- `git-lfs`（只有项目将来使用 Git LFS 时才需要）；
- `colcon`、`rosdep` 以及项目所需的其他依赖；
- Piper、Orbbec 相机和 YOLO 运行所需的驱动及环境。

### 7.2 将仓库克隆为工作区的 `src`

仓库是公开仓库时，使用 HTTPS 下载不需要配置 SSH：

```bash
mkdir -p /home/xie/wzl/elevate_ws
cd /home/xie/wzl/elevate_ws
git clone --recurse-submodules \
  https://github.com/xingoinglu/robotics_arms_elevate.git src
```

克隆后目录应为：

```text
/home/xie/wzl/elevate_ws/src
```

不要克隆成 `/home/xie/wzl/elevate_ws/src/robotics_arms_elevate`，否则会比原工作区多一层目录。

如果已经使用普通 `git clone` 下载，应补充初始化子模块：

```bash
cd /home/xie/wzl/elevate_ws/src
git submodule update --init --recursive
```

### 7.3 验证下载版本

```bash
cd /home/xie/wzl/elevate_ws/src
git remote -v
git branch --show-current
git log -1 --oneline
git status
git submodule status
```

接手人应把 `git rev-parse HEAD` 的结果与交接人提供的最终提交编号进行比较。

### 7.4 安装 ROS 依赖

```bash
cd /home/xie/wzl/elevate_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

`rosdep` 只能安装已经在 ROS 依赖系统中声明的软件包。相机驱动、机械臂驱动、YOLO/Conda 环境、模型和硬件权限仍可能需要按照各功能包的 `README.md` 单独配置。

### 7.5 编译工作区

```bash
cd /home/xie/wzl/elevate_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install_wzl/setup.bash
```

建议先在不连接或不使能机械臂的情况下完成编译和软件检查。涉及实机运动前，应阅读 `piper_launch/README.md` 并确认急停、工作空间和运动开关状态。

## 8. 接手人的后续更新流程

获取最新代码：

```bash
cd /home/xie/wzl/elevate_ws/src
git pull --rebase origin main
git submodule update --init --recursive
```

如果源代码发生变化，回到工作区根目录重新编译：

```bash
cd /home/xie/wzl/elevate_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install_wzl/setup.bash
```

如果接手人需要上传代码，建议配置 SSH，并确认拥有 GitHub 仓库写入权限。没有写入权限时，可以 Fork 仓库后通过 Pull Request 提交修改。

## 9. 推送被拒绝时的处理

出现下面的提示：

```text
! [rejected] main -> main (fetch first)
```

说明 GitHub 上存在本地尚未获取的新提交。先执行：

```bash
git pull --rebase origin main
git push origin main
```

如果发生冲突：

```bash
git status
```

手动修改冲突文件，确认最终内容后执行：

```bash
git add 冲突文件
git rebase --continue
```

所有冲突处理完成后再上传：

```bash
git push origin main
```

如果希望放弃这次 rebase 并返回操作前状态：

```bash
git rebase --abort
```

不要随意执行 `git push --force`，它可能覆盖其他人的远程提交。

## 10. 交接检查清单

### 交接人

- [ ] 所有需要交接的代码均已提交并推送。
- [ ] 本地 `main` 与 `origin/main` 一致。
- [ ] GitHub 页面可以看到最终提交。
- [ ] 没有提交密码、Token、私钥和隐私数据。
- [ ] 已提供最终提交编号和子模块版本。
- [ ] 已说明系统、ROS、Python/Conda 和硬件环境。
- [ ] 已说明模型、标定文件和本机配置的获取方式。
- [ ] 已记录启动方法、测试方法、未完成事项和已知问题。

### 接手人

- [ ] 已使用 `--recurse-submodules` 下载完整代码。
- [ ] 当前提交编号与交接版本一致。
- [ ] 已安装 ROS 和项目依赖。
- [ ] 工作区能够正常编译。
- [ ] 已阅读各功能包的 `README.md`。
- [ ] 已先完成安全的软件测试，再进行实机测试。
- [ ] 已获得仓库权限或确认 Pull Request 协作方式。

