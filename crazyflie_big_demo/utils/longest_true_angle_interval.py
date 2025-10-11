import numpy as np

def longest_true_angle_interval(angs, is_avail, ang_range_to_ignore=None):
    """
    计算 is_avail 连续 True 的最长角度区间（考虑 360° 周期），并给出中心角。
    可通过 ang_range_to_ignore=(start, end) 指定一个忽略区间，提前置 False。
    """
    angs = np.asarray(angs)
    is_avail = np.asarray(is_avail, dtype=bool)

    # 确保角度范围在 [0, 360)
    angs = np.mod(angs, 360)
    sort_idx = np.argsort(angs)
    angs = angs[sort_idx]
    is_avail = is_avail[sort_idx]

    # --- Step 1: 忽略指定角度区间 ---
    if ang_range_to_ignore is not None:
        start_ig, end_ig = np.mod(ang_range_to_ignore, 360)

        if start_ig <= end_ig:
            mask_ignore = (angs >= start_ig) & (angs <= end_ig)
        else:
            # 跨 0° 的情况，比如 (355, 10)
            mask_ignore = (angs >= start_ig) | (angs <= end_ig)

        is_avail[mask_ignore] = False

    # --- Step 2: 扩展一圈以考虑周期连续性 ---
    angs_ext = np.concatenate([angs, angs + 360])
    avail_ext = np.concatenate([is_avail, is_avail])

    # --- Step 3: 遍历寻找最长 True 区间 ---
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

    if best_start is None:  # 全 False
        return None, None, None

    # --- Step 4: 计算中心角 ---
    center = (best_start + (best_end - best_start) / 2) % 360

    # 限制在 [0, 360)
    best_start %= 360
    best_end %= 360

    return best_start, best_end, center


if __name__ == "__main__":
    angs = [350, 355, 0, 5, 10, 15, 100, 110, 120]
    is_avail = [False, True, True, True, True, True, False, True, True]
    ignore_range = (355, 10)

    start, end, center = longest_true_angle_interval(angs, is_avail, ignore_range)
    print(f"最长连续True区间：{start:.1f}° → {end:.1f}°，中心角：{center:.1f}°")
