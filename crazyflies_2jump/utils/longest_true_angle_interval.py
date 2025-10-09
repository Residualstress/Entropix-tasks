import numpy as np

def longest_true_angle_interval(angs, is_avail):
    """
    计算is_avail连续True的最长角度区间（考虑360°周期），并给出中心角
    """
    angs = np.asarray(angs)
    is_avail = np.asarray(is_avail, dtype=bool)

    # 确保角度范围在 [0, 360)
    angs = np.mod(angs, 360)
    sort_idx = np.argsort(angs)
    angs = angs[sort_idx]
    is_avail = is_avail[sort_idx]

    # 为了处理周期连续性，将数组拼接一遍 (延长一圈)
    angs_ext = np.concatenate([angs, angs + 360])
    avail_ext = np.concatenate([is_avail, is_avail])

    # 找出所有连续 True 段
    max_len = 0
    best_start = best_end = None
    i = 0
    n = len(angs_ext)
    while i < n:
        if avail_ext[i]:
            start = i
            while i < n and avail_ext[i]:
                i += 1
            end = i - 1
            length = angs_ext[end] - angs_ext[start]
            if length > max_len:
                max_len = length
                best_start = angs_ext[start]
                best_end = angs_ext[end]
        i += 1

    if best_start is None:  # 全False
        return None, None, None

    # 中心角（取周期平均）
    center = (best_start + (best_end - best_start)/2) % 360

    # 限制在 [0, 360)
    best_start %= 360
    best_end %= 360

    return best_start, best_end, center

if __name__ == "__main__":
    angs = [350, 355, 0, 5, 10, 15, 100, 110, 120]
    is_avail = [False, True, True, True, True, True, False, True, True]

    start, end, center = longest_true_angle_interval(angs, is_avail)
    print(f"最长连续True区间：{start:.1f}° → {end:.1f}°，中心角：{center:.1f}°")
