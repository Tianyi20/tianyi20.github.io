---
title: '每次终端启动时，Anaconda 会默认激活 base 环境。避免这个行为，同时保持 conda 的功能'
date: 2024-12-21
permalink: /posts/2024/12/blog-post-anaconda/
tags:
  - Anaconda

---

如果你的 `~/.bashrc` 文件中只有这一行：

```bash
source ~/anaconda3/bin/activate
```

那么每次终端启动时，Anaconda 会默认激活 `base` 环境。为了避免这个行为，同时保持 `conda` 的功能，可以按以下步骤修改：

---

### 1. 修改 `~/.bashrc` 文件
将这行代码修改为只加载 `conda`，而不激活 `base` 环境。

#### 打开并编辑 `~/.bashrc`：
```bash
nano ~/.bashrc
```

#### 替换这行：
```bash
source ~/anaconda3/bin/activate
```

#### 改为以下代码：
```bash
# Initialize conda without activating the base environment
if [ -f "$HOME/anaconda3/etc/profile.d/conda.sh" ]; then
    . "$HOME/anaconda3/etc/profile.d/conda.sh"
else
    export PATH="$HOME/anaconda3/bin:$PATH"
fi
```

---

### 2. 禁用自动激活 `base` 环境
运行以下命令，确保 Conda 不会自动激活 `base` 环境：
```bash
conda config --set auto_activate_base false
```

---

### 3. 验证修改
1. 重新加载 `.bashrc` 文件：
   ```bash
   source ~/.bashrc
   ```

2. 打开一个新的终端窗口，你应该看到：
   - 不再自动进入 `(base)` 环境；
   - `conda` 命令仍然可用。

3. 如果需要进入 `base` 环境，可以手动激活：
   ```bash
   conda activate base
   ```

---

### 结果
通过以上修改，你的终端不会自动进入 `(base)` 环境，但可以随时手动激活环境并使用 Conda 的功能。
