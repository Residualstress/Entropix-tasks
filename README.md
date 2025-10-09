# 任务保存架构
Entropix/
Task_001_<task_name>/
  env/
    requirements.txt
    environment.yml
    setup_instructions.txt
  src/
    main.py
    utils.py
    config.py
  data/
    input_example.json
  output/
  README.txt
  LICENSE 或 notes.md
  
各目录/文件说明
	•	env/ — 存放环境配置相关文件
	•	requirements.txt：使用 pip 的依赖清单
	•	environment.yml：使用 conda 的环境文件（可选）
	•	setup_instructions.txt：简要的环境搭建说明（可选）
	•	src/ — 源代码目录
	•	main.py：主程序入口
	•	utils.py：工具函数集合
	•	config.py：配置与参数定义
	•	其他模块文件…
	•	data/ — 示例或输入数据目录
	•	input_example.json：示例输入数据
	•	output/ — 任务运行生成的输出或模型结果
	•	README.txt 或 README.md — 该任务的说明文档
	•	LICENSE 或 notes.md（可选） — 许可证或备注文件

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
