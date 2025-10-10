from pathlib import Path
import os, time

def write_flag_atomic(path: Path, value: int):
    tmp = path.with_suffix(path.suffix + ".tmp")
    # 1) 先写到临时文件
    with tmp.open("w", encoding="utf-8") as f:
        f.write("1\n" if value else "0\n")
        f.flush()
        os.fsync(f.fileno())  # 尽量把数据落盘（Windows/Unix 都可用）
    # 2) 再用原子替换
    os.replace(tmp, path)  # 原子操作：读端不是看到旧文件就是新文件

# 用法示例：
p = Path(__file__).with_name("parameters.txt")
write_flag_atomic(p, 1)  # 写 1
time.sleep(5)
write_flag_atomic(p, 0)  # 写 0
