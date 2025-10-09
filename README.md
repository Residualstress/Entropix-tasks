Entropix/
├── Task_001_<task_name>/
│   ├── env/
│   │   ├── requirements.txt        # Python环境依赖导出文件（pip）
│   │   ├── environment.yml         # Conda环境文件（如有）
│   │   └── setup_instructions.txt  # 简要环境搭建步骤（如需）
│   │
│   ├── src/
│   │   ├── main.py                 # 主运行脚本
│   │   ├── utils.py                # 工具函数
│   │   ├── config.py               # 配置文件（参数、路径等）
│   │   └── ...                     # 其他模块代码
│   │
│   ├── data/                       # 可选：输入或示例数据
│   │   ├── input_example.json
│   │   └── ...
│   │
│   ├── output/                     # 可选：输出结果或模型
│   │   └── ...
│   │
│   ├── README.txt                  # 任务说明文档
│   └── LICENSE / notes.md (可选)

# Task 名称: <task_name>

## 1. 简介
说明该任务的目的与功能。例如：
本任务用于实现 Entropix 飞行器姿态控制算法的初步仿真。

## 2. 环境配置
Python 版本: 3.11
主要依赖:
- numpy==1.26.4
- matplotlib==3.8.4
- torch==2.3.0
导出方式：
    pip freeze > env/requirements.txt
    conda env export > env/environment.yml

## 3. 文件结构
说明 src、data、output 文件夹中内容的作用。

## 4. 使用方法
运行方式：
    cd src
    python main.py --config config.py
或通过 makefile/脚本运行。

## 5. 优化与备注
- 可替换参数: ...
- 未来改进方向: ...
- 当前问题或限制: ...
