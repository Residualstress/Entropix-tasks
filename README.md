# 任务保存架构

Entropix/  
Task_001\_<task_name>/  
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
  README.txt 或 README.md  
  LICENSE 或 notes.md  

---

## 各目录 / 文件说明

- **env/** — 存放环境配置相关文件  
  - `requirements.txt`：使用 pip 的依赖清单  
  - `environment.yml`：使用 conda 的环境文件（可选）  
  - `setup_instructions.txt`：简要的环境搭建说明（可选）  

- **src/** — 源代码目录  
  - `main.py`：主程序入口  
  - `utils.py`：工具函数集合  
  - `config.py`：配置与参数定义  
  - 其他模块文件…  

- **data/** — 示例或输入数据目录  
  - `input_example.json`：示例输入数据  

- **output/** — 任务运行生成的输出或模型结果  

- **README.txt** 或 **README.md** — 该任务的说明文档  

- **LICENSE** 或 **notes.md**（可选） — 许可证或备注文件  
