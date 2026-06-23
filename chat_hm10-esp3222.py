import time
import sys
import threading
import abc
import csv
import logging
import re
from typing import Optional, Tuple, cast
import requests
import socketio
from hm10_esp32 import HM10ESP32Bridge
import csv

def solve_optimal_maze_strategy3(start_node=25, time_limit=60.0):
    """
    在給定時間限制 (秒) 內，最大化訪問死路的得分。
    得分 = 離起點的曼哈頓距離 * 10。
    """
    # --- 調整時間權重區 (秒) ---
    time_f = 0.5  # 直走 (forward)
    time_r = 1.3  # 右轉 (right)
    time_l = 1.3  # 左轉 (left)
    time_b = 1.9  # 迴轉 (back, 包含 b 與 B)
    # --------------------------

    nlist = {}
    excel_map = {1: "North", 2: "South", 3: "West", 4: "East"}
    dir_map = {1: "North", 2: "East", 3: "South", 4: "West"}
    r_map = {v: k for k, v in dir_map.items()}
    
    # 1. 讀取與建立地圖
    try:
        filename = "big_maze.csv"
        with open(filename, newline='', encoding='utf-8') as f:
            reader = csv.reader(f)
            next(reader)
            for row in reader:
                if not row: continue
                node_index = int(float(row[0]))
                neighbors = {int(float(row[i])): excel_map[i] for i in range(1, 5) if row[i] and row[i].strip()}
                nlist[node_index] = neighbors
    except FileNotFoundError:
        print(f"錯誤：找不到 {filename}")
        return ""

    # 2. BFS 計算座標與矩形邊界 (以 start_node 為原點)
    coords = {start_node: (0, 0)}
    queue = [start_node]
    visited = {start_node}
    while queue:
        curr = queue.pop(0)
        cx, cy = coords[curr]
        for neighbor, direction in nlist.get(curr, {}).items():
            if neighbor not in visited:
                if direction == "North": coords[neighbor] = (cx, cy + 1)
                elif direction == "South": coords[neighbor] = (cx, cy - 1)
                elif direction == "West": coords[neighbor] = (cx - 1, cy)
                elif direction == "East": coords[neighbor] = (cx + 1, cy)
                visited.add(neighbor)
                queue.append(neighbor)
    
    # 計算邊界用於判斷 B
    all_x = [c[0] for c in coords.values()]
    all_y = [c[1] for c in coords.values()]
    min_x, max_x = min(all_x), max(all_x)
    min_y, max_y = min(all_y), max(all_y)

    # 3. 定義死路與精準的分數計算
    dead_ends = [n for n, neigh in nlist.items() if len(neigh) == 1]
    
    # 【修正】：確保分數嚴格等於「距離起點的曼哈頓距離 * 10」
    de_scores = {}
    start_x, start_y = coords[start_node]
    for de in dead_ends:
        dist = abs(coords[de][0] - start_x) + abs(coords[de][1] - start_y)
        de_scores[de] = dist * 10
    
    # 尋路 BFS 函式
    def get_path_info(start, end):
        q = [(start, [], [])]
        v = {start}
        while q:
            curr, p_nodes, p_moves = q.pop(0)
            if curr == end: return p_nodes, p_moves
            for neighbor, move_dir in nlist.get(curr, {}).items():
                if neighbor not in v:
                    v.add(neighbor)
                    q.append((neighbor, p_nodes + [neighbor], p_moves + [move_dir]))
        return [], []

    # 排除起點（若起點是死路，曼哈頓距離為 0，無需作為額外目標）
    targets = [de for de in dead_ends if de != start_node]
    N = len(targets)
    if N == 0: return ""

    # 預處理所有目標點之間的路徑
    paths = {}
    for i in range(N):
        paths[("start", i)] = get_path_info(start_node, targets[i])
        for j in range(N):
            if i != j:
                paths[(i, j)] = get_path_info(targets[i], targets[j])

    # 計算路徑耗時 (自動計算所有 f, l, r, b 的時間)
    def eval_time(moves, start_facing):
        t = 0.0
        cardir = start_facing
        for m in moves:
            sub = (r_map[m] - r_map[cardir]) % 4
            t += [time_f, time_r, time_b, time_l][sub]
            cardir = m
        return t

    # 預計算時間成本矩陣
    arrive_facing = {i: paths[("start", i)][1][-1] for i in range(N)}
    start_costs = []
    for i in range(N):
        # 為了保證第一步是 f，初始面向設為第一步的移動方向
        first_m = paths[("start", i)][1][0] if paths[("start", i)][1] else "West"
        start_costs.append(eval_time(paths[("start", i)][1], first_m))

    adj_costs = [[0.0]*N for _ in range(N)]
    for i in range(N):
        for j in range(N):
            if i != j:
                # 死路原路退出時的第一步必為反向，BFS 會自然產生 b，時間會自動加上 time_b
                adj_costs[i][j] = eval_time(paths[(i, j)][1], arrive_facing[i])

    # 4. Bitmask DP 求解最短時間
    # dp[mask][i] = (最短時間, 前一個節點索引)
    dp = [[(float('inf'), -1)] * N for _ in range(1 << N)]
    for i in range(N):
        dp[1 << i][i] = (start_costs[i], -1)

    for mask in range(1, 1 << N):
        for i in range(N):
            if dp[mask][i][0] == float('inf'): continue
            for j in range(N):
                if not (mask & (1 << j)):
                    new_mask = mask | (1 << j)
                    new_time = dp[mask][i][0] + adj_costs[i][j]
                    if new_time < dp[new_mask][j][0]:
                        dp[new_mask][j] = (new_time, i)

    # 5. 挑選時間限制內得分最高的走法
    best_score = -1
    best_time = float('inf')
    best_mask = -1
    best_last = -1

    for mask in range(1, 1 << N):
        # 計算此組合的總分數
        current_score = sum(de_scores[targets[k]] for k in range(N) if (mask & (1 << k)))
        for i in range(N):
            # 必須加上最後一個死路的離開迴轉時間
            total_time = dp[mask][i][0] + time_b
            if total_time <= time_limit:
                # 優先選高分；同分則選最省時的
                if current_score > best_score:
                    best_score = current_score
                    best_time = total_time
                    best_mask, best_last = mask, i
                elif current_score == best_score and total_time < best_time:
                    best_time = total_time
                    best_mask, best_last = mask, i

    if best_mask == -1:
        print(f"在 {time_limit} 秒內無法到達任何死路。")
        return "bllfrbfBrffrbflfBfrfblfflBfblfrr" * 100

    # 6. 回溯路徑順序
    seq = []
    curr_m, curr_l = best_mask, best_last
    while curr_l != -1:
        seq.append(curr_l)
        prev = dp[curr_m][curr_l][1]
        curr_m ^= (1 << curr_l)
        curr_l = prev
    seq.reverse()

    # 7. 組合指令序列
    final_moves = []
    node_seq = [start_node]
    dead_ends_visited_order = []
    
    # 第一段：起點到首站
    final_moves.extend(paths[("start", seq[0])][1])
    node_seq.extend(paths[("start", seq[0])][0])
    dead_ends_visited_order.append(targets[seq[0]])

    # 中間段
    for k in range(len(seq)-1):
        i, j = seq[k], seq[k+1]
        final_moves.extend(paths[(i, j)][1])
        node_seq.extend(paths[(i, j)][0])
        dead_ends_visited_order.append(targets[j])

    # 補齊最後一個死路的強制迴轉
    last_f = final_moves[-1]
    opp_dir = dir_map[(r_map[last_f] + 1) % 4 + 1] # 反轉 180 度
    final_moves.append(opp_dir)
    node_seq.append(targets[seq[-1]])

    # 8. 轉換指令與 B 判斷
    t_map = {0: "f", 1: "r", 2: "b", 3: "l"}
    initial_facing = final_moves[0]
    cardir = initial_facing
    t_string = ""
    
    for i, move in enumerate(final_moves):
        sub = (r_map[move] - r_map[cardir]) % 4
        action = t_map[sub]
        
        # 嚴格的邊界判斷邏輯
        if action == 'b':
            curr_pos = coords[node_seq[i]]
            is_out = False
            if cardir == "North": is_out = (curr_pos[0] + 1 > max_x)
            elif cardir == "South": is_out = (curr_pos[0] - 1 < min_x)
            elif cardir == "East": is_out = (curr_pos[1] - 1 < min_y)
            elif cardir == "West": is_out = (curr_pos[1] + 1 > max_y)
            
            if is_out: 
                action = 'B'
                
        t_string += action
        cardir = move

    # 輸出統計資訊
    print(f"--- 規劃結果 ---")
    print(f"時間限制: {time_limit} 秒 | 預估總時間: {best_time:.2f} 秒")
    print(f"訪問死路順序: {dead_ends_visited_order}")
    print(f"總獲得分數: {best_score*3}")
    print(f"指令序列長度: {len(t_string)}")
    print(f"指令序列 (t): {t_string}")

    # 補足長度
    # t_string += ("bllfrbfBrffrbflfBfrfblfflBfblfrr" * 100)
    return t_string

# 呼叫範例：
# result = solve_optimal_maze_strategy(start_node=1, time_limit=120.0)

def solve_optimal_maze_strategy():
    nlist = {}
    excel_map = {1: "North", 2: "South", 3: "West", 4: "East"}
    dir_map = {1: "North", 2: "East", 3: "South", 4: "West"}
    r_map = {v: k for k, v in dir_map.items()}
    
    # 1. 讀取與建立地圖
    try:
        filename = "medium_maze.csv"
        with open(filename, newline='', encoding='utf-8') as f:
            reader = csv.reader(f)
            next(reader)
            for row in reader:
                if not row: continue
                node_index = int(float(row[0]))
                neighbors = {int(float(row[i])): excel_map[i] for i in range(1, 5) if row[i].strip()}
                nlist[node_index] = neighbors
    except FileNotFoundError:
        print(f"錯誤：找不到 {filename}")
        return ""

    # 2. BFS 計算所有節點的 (x, y) 座標與邊界
    start_node = 1
    coords = {start_node: (0, 0)}
    queue = [start_node]
    visited = {start_node}
    while queue:
        curr = queue.pop(0)
        cx, cy = coords[curr]
        for neighbor, direction in nlist.get(curr, {}).items():
            if neighbor not in visited:
                if direction == "North": coords[neighbor] = (cx, cy + 1)
                elif direction == "South": coords[neighbor] = (cx, cy - 1)
                elif direction == "West": coords[neighbor] = (cx - 1, cy)
                elif direction == "East": coords[neighbor] = (cx + 1, cy)
                visited.add(neighbor)
                queue.append(neighbor)
    
    all_x = [c[0] for c in coords.values()]
    all_y = [c[1] for c in coords.values()]
    min_x, max_x = min(all_x), max(all_x)
    min_y, max_y = min(all_y), max(all_y)

    # 3. 路徑規劃 (完全還原你的 BFS 尋路)
    dead_ends = [n for n, neigh in nlist.items() if len(neigh) == 1]
    
    def get_path_info(start, end):
        q = [(start, [], [])]
        v = {start}
        while q:
            curr, p_nodes, p_moves = q.pop(0)
            if curr == end: return p_nodes, p_moves
            for neighbor, move_dir in nlist.get(curr, {}).items():
                if neighbor not in v:
                    v.add(neighbor)
                    q.append((neighbor, p_nodes + [neighbor], p_moves + [move_dir]))
        return None, None

    current_node = start_node
    unvisited_de = set(dead_ends)
    dead_ends_visited_order = []
    
    if current_node in unvisited_de:
        unvisited_de.remove(current_node)
        dead_ends_visited_order.append(current_node)

    unvisited_list = list(unvisited_de)
    N = len(unvisited_list)

    # === DP 預處理：計算所有點到點的純步數 ===
    paths = {}
    for i in range(N):
        p_nodes, p_moves = get_path_info(current_node, unvisited_list[i])
        paths[("start", i)] = (p_nodes, p_moves)
        for j in range(N):
            if i != j:
                p_nodes_ij, p_moves_ij = get_path_info(unvisited_list[i], unvisited_list[j])
                paths[(i, j)] = (p_nodes_ij, p_moves_ij)

    # dp[mask][i] = (min_steps, prev_i)
    dp = [[(float('inf'), -1)] * N for _ in range(1 << N)]
    
    for i in range(N):
        if paths[("start", i)][1] is not None:
            dp[1 << i][i] = (len(paths[("start", i)][1]), -1)
    
    # DP 狀態轉移 (只看步數，不看轉向成本)
    for mask in range(1, 1 << N):
        for i in range(N):
            if dp[mask][i][0] == float('inf'): continue
            for j in range(N):
                if not (mask & (1 << j)):
                    nxt_mask = mask | (1 << j)
                    if paths[(i, j)][1] is not None:
                        steps = len(paths[(i, j)][1])
                        if dp[mask][i][0] + steps < dp[nxt_mask][j][0]:
                            dp[nxt_mask][j] = (dp[mask][i][0] + steps, i)

    # === 回溯最佳路徑順序 ===
    best_steps = float('inf')
    best_last = -1
    for i in range(N):
        if dp[(1 << N) - 1][i][0] < best_steps:
            best_steps = dp[(1 << N) - 1][i][0]
            best_last = i

    curr_mask = (1 << N) - 1
    curr_idx = best_last
    seq = []
    while curr_idx != -1:
        seq.append(curr_idx)
        prev_idx = dp[curr_mask][curr_idx][1]
        curr_mask = curr_mask ^ (1 << curr_idx)
        curr_idx = prev_idx
    seq.reverse()

    # 將最佳順序組合成完整的移動指令
    final_moves = []
    node_sequence = [current_node]
    curr_idx_state = "start"
    
    for idx in seq:
        target = unvisited_list[idx]
        p_nodes, p_moves = paths[(curr_idx_state, idx)]
        final_moves.extend(p_moves)
        for n in p_nodes:
            node_sequence.append(n)
        curr_idx_state = idx
        dead_ends_visited_order.append(target)

    # 4. 指令轉換與 B 判斷
    t_map = {0: "f", 1: "r", 2: "b", 3: "l"}
    
    # 【關鍵解法 1】：強迫初始面向等於第一步的方向，確保第一個動作永遠產出 'f'
    initial_facing = final_moves[0] if final_moves else "West"
    cardir = initial_facing
    t_string = ""
    
    for i, move in enumerate(final_moves):
        prev_idx = r_map[cardir]
        nex_idx = r_map[move]
        sub = (nex_idx - prev_idx) % 4
        action = t_map[sub]
        
        # 【關鍵解法 2】：因為死路原路進出，sub 必為 2，必定產出 'b'
        if action == 'b':
            curr_pos = coords[node_sequence[i]]
            is_out = False
            # 判斷機器人右側是否超出地圖邊界範圍
            if cardir == "North":   
                if curr_pos[0] + 1 > max_x: is_out = True
            elif cardir == "South": 
                if curr_pos[0] - 1 < min_x: is_out = True
            elif cardir == "East":  
                if curr_pos[1] - 1 < min_y: is_out = True
            elif cardir == "West":  
                if curr_pos[1] + 1 > max_y: is_out = True
            
            if is_out:
                action = 'B'
        
        t_string += action
        cardir = move

    # 印出結果
    print(f"經過死路的順序: {dead_ends_visited_order}")
    print(f"指令序列 (t): {t_string}")
    print(f"總最少步數: {best_steps} | 指令長度: {len(t_string)}")
    
    # 補足指令
    filler = "bllfrbfBrffrbflfBfrfblfflBfblfrr"
    t_string += filler * 100
        
    return t_string

# import csv

def solve_optimal_maze_strategy2():
    # --- 調整權重區 (可調整直走與左右轉的代價) ---
    cost_f = 1.0  # 直走
    cost_r = 2.5  # 右轉
    cost_l = 2.5  # 左轉
    # 迴轉 (b/B) cost 設為 0，因為死路迴轉次數固定，不影響 DP 最優路徑選擇
    # ------------------------------------------

    nlist = {}
    excel_map = {1: "North", 2: "South", 3: "West", 4: "East"}
    dir_map = {1: "North", 2: "East", 3: "South", 4: "West"}
    r_map = {v: k for k, v in dir_map.items()}
    
    # 1. 讀取與建立地圖
    try:
        filename = "medium_maze.csv"
        with open(filename, newline='', encoding='utf-8') as f:
            reader = csv.reader(f)
            next(reader)
            for row in reader:
                if not row: continue
                node_index = int(float(row[0]))
                neighbors = {int(float(row[i])): excel_map[i] for i in range(1, 5) if row[i].strip()}
                nlist[node_index] = neighbors
    except FileNotFoundError:
        print(f"錯誤：找不到 {filename}")
        return ""

    # 2. BFS 計算所有節點的 (x, y) 座標與邊界
    start_node = 1
    coords = {start_node: (0, 0)}
    queue = [start_node]
    visited = {start_node}
    while queue:
        curr = queue.pop(0)
        cx, cy = coords[curr]
        for neighbor, direction in nlist.get(curr, {}).items():
            if neighbor not in visited:
                if direction == "North": coords[neighbor] = (cx, cy + 1)
                elif direction == "South": coords[neighbor] = (cx, cy - 1)
                elif direction == "West": coords[neighbor] = (cx - 1, cy)
                elif direction == "East": coords[neighbor] = (cx + 1, cy)
                visited.add(neighbor)
                queue.append(neighbor)
    
    all_x = [c[0] for c in coords.values()]
    all_y = [c[1] for c in coords.values()]
    min_x, max_x = min(all_x), max(all_x)
    min_y, max_y = min(all_y), max(all_y)

    # 3. 尋路與 DP 規劃 (完全保留你的 BFS 邏輯)
    dead_ends = [n for n, neigh in nlist.items() if len(neigh) == 1]
    
    def get_path_info(start, end):
        q = [(start, [], [])]
        v = {start}
        while q:
            curr, p_nodes, p_moves = q.pop(0)
            if curr == end: return p_nodes, p_moves
            for neighbor, move_dir in nlist.get(curr, {}).items():
                if neighbor not in v:
                    v.add(neighbor)
                    q.append((neighbor, p_nodes + [neighbor], p_moves + [move_dir]))
        return [], []

    current_node = start_node
    unvisited_de = set(dead_ends)
    dead_ends_visited_order = []
    
    if current_node in unvisited_de:
        unvisited_de.remove(current_node)
        dead_ends_visited_order.append(current_node)

    unvisited_list = list(unvisited_de)
    N = len(unvisited_list)

    # === DP 預處理：計算所有點到點的 BFS 路徑 ===
    paths = {}
    for i in range(N):
        p_nodes, p_moves = get_path_info(current_node, unvisited_list[i])
        paths[("start", i)] = (p_nodes, p_moves)
        for j in range(N):
            if i != j:
                p_nodes_ij, p_moves_ij = get_path_info(unvisited_list[i], unvisited_list[j])
                paths[(i, j)] = (p_nodes_ij, p_moves_ij)

    # 定義評估加權成本函式 (將 b 的 cost 設為 0)
    def eval_cost(u_idx, v_idx, start_facing):
        p_moves = paths[(u_idx, v_idx)][1]
        cost = 0
        cardir = start_facing
        for move in p_moves:
            sub = (r_map[move] - r_map[cardir]) % 4
            a_cost = [cost_f, cost_r, 0.0, cost_l][sub] # 這裡 b 的 cost 是 0
            cost += a_cost
            cardir = move
        return cost

    # 計算抵達每個死路時的面向 (因為死路只有一條路，最後一步的方向固定)
    arrive_facing = {}
    for i in range(N):
        p_moves = paths[("start", i)][1]
        arrive_facing[i] = p_moves[-1] if p_moves else "West"

    # 預計算 DP 的成本矩陣
    cost_matrix = {}
    for i in range(N):
        # 起點到第一站：保證第一步是 'f'，所以初始面向設為第一步的方向
        first_move = paths[("start", i)][1][0] if paths[("start", i)][1] else "West"
        cost_matrix[("start", i)] = eval_cost("start", i, first_move)
        
        for j in range(N):
            if i != j:
                # 從死路 i 離開到 j：起點面向即為剛抵達 i 時的面向
                cost_matrix[(i, j)] = eval_cost(i, j, arrive_facing[i])

    # dp[mask][i] = (min_cost, prev_i)
    dp = [[(float('inf'), -1)] * N for _ in range(1 << N)]
    
    for i in range(N):
        dp[1 << i][i] = (cost_matrix[("start", i)], -1)
    
    # === DP 狀態轉移 (計算最小加權成本) ===
    for mask in range(1, 1 << N):
        for i in range(N):
            if dp[mask][i][0] == float('inf'): continue
            for j in range(N):
                if not (mask & (1 << j)):
                    nxt_mask = mask | (1 << j)
                    c = cost_matrix[(i, j)]
                    if dp[mask][i][0] + c < dp[nxt_mask][j][0]:
                        dp[nxt_mask][j] = (dp[mask][i][0] + c, i)

    # === 回溯最佳路徑順序 ===
    best_cost = float('inf')
    best_last = -1
    for i in range(N):
        if dp[(1 << N) - 1][i][0] < best_cost:
            best_cost = dp[(1 << N) - 1][i][0]
            best_last = i

    curr_mask = (1 << N) - 1
    curr_idx = best_last
    seq = []
    while curr_idx != -1:
        seq.append(curr_idx)
        prev_idx = dp[curr_mask][curr_idx][1]
        curr_mask = curr_mask ^ (1 << curr_idx)
        curr_idx = prev_idx
    seq.reverse()

    # 將最佳順序組合成完整的移動指令與節點序列
    final_moves = []
    node_sequence = [current_node]
    curr_idx_state = "start"
    
    for idx in seq:
        target = unvisited_list[idx]
        p_nodes, p_moves = paths[(curr_idx_state, idx)]
        final_moves.extend(p_moves)
        for n in p_nodes:
            node_sequence.append(n)
        curr_idx_state = idx
        dead_ends_visited_order.append(target)

    # 4. 指令轉換與 B 判斷
    t_map = {0: "f", 1: "r", 2: "b", 3: "l"}
    
    # 【關鍵解法】：強迫初始面向等於第一步的方向，確保第一個動作永遠產出 'f'
    initial_facing = final_moves[0] if final_moves else "West"
    cardir = initial_facing
    t_string = ""
    
    for i, move in enumerate(final_moves):
        prev_idx = r_map[cardir]
        nex_idx = r_map[move]
        sub = (nex_idx - prev_idx) % 4
        action = t_map[sub]
        
        # B 判斷邏輯 (依照原先需求，嚴格判斷右側是否超界)
        if action == 'b':
            curr_pos = coords[node_sequence[i]]
            is_out = False
            if cardir == "North":   
                if curr_pos[0] + 1 > max_x: is_out = True
            elif cardir == "South": 
                if curr_pos[0] - 1 < min_x: is_out = True
            elif cardir == "East":  
                if curr_pos[1] - 1 < min_y: is_out = True
            elif cardir == "West":  
                if curr_pos[1] + 1 > max_y: is_out = True
            
            if is_out:
                action = 'B'
        
        t_string += action
        cardir = move

    # 印出結果
    print(f"經過死路的順序: {dead_ends_visited_order}")
    print(f"指令序列 (t): {t_string}")
    print(f"總加權成本: {best_cost} | 指令長度: {len(t_string)}")
    
    # 補足指令
    filler = "bllfrbfBrffrbflfBfrfblfflBfblfrr"
    t_string += filler * 100
        
    return t_string

def solve_dynamic_maze_strategy():
    nlist = {}
    excel_map = {1: "North", 2: "South", 3: "West", 4: "East"}
    dir_map = {1: "North", 2: "East", 3: "South", 4: "West"}
    r_map = {v: k for k, v in dir_map.items()}
    
    # 1. 讀取與建立地圖
    try:
        filename = "medium_maze.csv"
        with open(filename, newline='', encoding='utf-8') as f:
            reader = csv.reader(f)
            next(reader)
            for row in reader:
                if not row: continue
                node_index = int(float(row[0]))
                neighbors = {int(float(row[i])): excel_map[i] for i in range(1, 5) if row[i].strip()}
                nlist[node_index] = neighbors
    except FileNotFoundError:
        print(f"錯誤：找不到 {filename}")
        return ""

    # 2. BFS 計算所有節點的 (x, y) 座標與邊界
    start_node = 1
    coords = {start_node: (0, 0)}
    queue = [start_node]
    visited = {start_node}
    while queue:
        curr = queue.pop(0)
        cx, cy = coords[curr]
        for neighbor, direction in nlist.get(curr, {}).items():
            if neighbor not in visited:
                if direction == "North": coords[neighbor] = (cx, cy + 1)
                elif direction == "South": coords[neighbor] = (cx, cy - 1)
                elif direction == "West": coords[neighbor] = (cx - 1, cy)
                elif direction == "East": coords[neighbor] = (cx + 1, cy)
                visited.add(neighbor)
                queue.append(neighbor)
    
    all_x = [c[0] for c in coords.values()]
    all_y = [c[1] for c in coords.values()]
    min_x, max_x = min(all_x), max(all_x)
    min_y, max_y = min(all_y), max(all_y)

    # 3. 路徑規劃
    dead_ends = [n for n, neigh in nlist.items() if len(neigh) == 1]
    scores = {de: abs(coords[de][0]) + abs(coords[de][1]) for de in dead_ends}
    
    def get_path_info(start, end):
        q = [(start, [], [])]
        v = {start}
        while q:
            curr, p_nodes, p_moves = q.pop(0)
            if curr == end: return p_nodes, p_moves
            for neighbor, move_dir in nlist.get(curr, {}).items():
                if neighbor not in v:
                    v.add(neighbor)
                    q.append((neighbor, p_nodes + [neighbor], p_moves + [move_dir]))
        return None, None

    current_node = start_node
    remaining_steps = 6767
    total_score = 0
    unvisited_de = set(dead_ends)
    dead_ends_visited_order = []
    
    # 初始狀態設定
    initial_facing = "West"
    if current_node in unvisited_de:
        neighbor_nodes = list(nlist[current_node].keys())
        initial_facing = nlist[current_node][neighbor_nodes[0]]
        total_score += scores[current_node]
        unvisited_de.remove(current_node)
        dead_ends_visited_order.append(current_node)

    final_moves = []
    node_sequence = [current_node]
    
    while unvisited_de and remaining_steps > 0:
        best_target, best_ratio, best_path = None, -1, None
        for target in unvisited_de:
            p_nodes, p_moves = get_path_info(current_node, target)
            if p_moves:
                ratio = scores[target] / len(p_moves)
                if len(p_moves) <= remaining_steps and ratio > best_ratio:
                    best_ratio, best_target, best_path = ratio, target, (p_nodes, p_moves)
        if not best_target: break
        final_moves.extend(best_path[1])
        for n in best_path[0]: node_sequence.append(n)
        remaining_steps -= len(best_path[1])
        total_score += scores[best_target]
        unvisited_de.remove(best_target)
        current_node = best_target
        dead_ends_visited_order.append(best_target)

    # 4. 指令轉換與 B 判斷
    t_map = {0: "f", 1: "r", 2: "b", 3: "l"}
    cardir = initial_facing
    t_string = ""
    
    for i, move in enumerate(final_moves):
        prev_idx = r_map[cardir]
        nex_idx = r_map[move]
        sub = (nex_idx - prev_idx) % 4
        action = t_map[sub]
        
        if action == 'b':
            curr_pos = coords[node_sequence[i]]
            is_out = False
            # 判斷機器人右側是否超出地圖邊界範圍
            if cardir == "North":   # 面向北，右邊是東，X 座標+1
                if curr_pos[0] + 1 > max_x: is_out = True
            elif cardir == "South": # 面向南，右邊是西，X 座標-1
                if curr_pos[0] - 1 < min_x: is_out = True
            elif cardir == "East":  # 面向東，右邊是南，Y 座標-1
                if curr_pos[1] - 1 < min_y: is_out = True
            elif cardir == "West":  # 面向西，右邊是北，Y 座標+1
                if curr_pos[1] + 1 > max_y: is_out = True
            
            if is_out:
                action = 'B'
        
        t_string += action
        cardir = move

    # 印出結果
    print(f"經過死路的順序: {dead_ends_visited_order}")
    print(f"指令序列 (t): {t_string}")
    print(f"總得分: {total_score} | 指令長度: {len(t_string)}")
    
    # 補足指令
    filler = "bllfrbfBrffrbflfBfrfblfflBfblfrr"
    t_string += filler * 100
        
    return t_string

# --- Scoreboard 類別 (維持不變) ---
log = logging.getLogger("scoreboard")

class ScoreboardServer:
    def __init__(self, teamname: str, host="http://carcar.ntuee.org/scoreboard", debug=False):
        self.teamname = teamname
        self.ip = host
        self.socket = socketio.Client(logger=debug, engineio_logger=debug)
        self.socket.register_namespace(TeamNamespace("/team"))
        self.socket.connect(self.ip, socketio_path="scoreboard.io")
        self.sid = self.socket.get_sid(namespace="/team")
        self._start_game(self.teamname)

    def _start_game(self, teamname: str):
        res = self.socket.call("start_game", {"teamname": teamname}, namespace="/team")
        log.info(res)

    def add_UID(self, UID_str: str) -> Tuple[int, float]:
        if not re.match(r"^[0-9A-Fa-f]{8}$", UID_str): return 0, 0
        res = self.socket.call("add_UID", UID_str, namespace="/team")
        return (res.get("score", 0), res.get("time_remaining", 0)) if res else (0, 0)

    def get_current_score(self) -> Optional[int]:
        try:
            res = requests.get(self.ip + "/current_score", params={"sid": self.sid})
            return res.json()["current_score"]
        except: return None

class TeamNamespace(socketio.ClientNamespace):
    def on_connect(self): log.info("Connected to scoreboard")
    def on_disconnect(self): log.info("Disconnected")

# --- 全域變數與監聽 (維持不變) ---
PORT = '/dev/ttyUSB1'
EXPECTED_NAME = 'diaob'
# path_commands = solve_dynamic_maze_strategy()
path_commands = solve_optimal_maze_strategy3()
sent = False
cnt=0

def background_listener(bridge, team_name, server_url):
    global sent
    global cnt
    scoreboard = None 
    while True:
        msg = bridge.listen()
        if msg:
            print(f"\r[HM10]: {msg}")
            if len(msg)==10:
                msgg=msg[:7]
                if(msg.startswith("nxt")):
                    msgg=msg[3:]
                for i in range(3):
                    if cnt<len(path_commands):
                        cur+=path_commands[cnt]
                        cnt=cnt+1
                bridge.send(cur+'\n')
                if scoreboard and len(msgg) == 7:
                    cc=['0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F']
                    for c in cc:
                        score, _ = scoreboard.add_UID(msgg+c)
                        print(f"得分: {score} | 目前總分: {scoreboard.get_current_score()}")
            # 處理 UID 加分
            if scoreboard and len(msg) == 7:
                cc=['0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F']
                for c in cc:
                    score, _ = scoreboard.add_UID(msg+c)
                    print(f"得分: {score} | 目前總分: {scoreboard.get_current_score()}")
            
            # 偵測啟動訊號
            if not sent and msg.startswith("stby"):
                cur=""
                for i in range(3):
                    if cnt<len(path_commands):
                        cur+=path_commands[cnt]
                        cnt=cnt+1
                
                print("收到 stby，啟動計分板...")
                scoreboard = ScoreboardServer(team_name, server_url)
                bridge.send(cur+'\n')
                sent = True
            if msg.startswith("nxt"):
                cur=""
                for i in range(3):
                    if cnt<len(path_commands):
                        cur+=path_commands[cnt]
                        cnt=cnt+1
                bridge.send(cur+'\n')

def main():

    bridge = HM10ESP32Bridge(port=PORT)

    # 1. Configuration Check

    current_name = bridge.get_hm10_name()

    if current_name != EXPECTED_NAME:

        print(f"Target mismatch. Current: {current_name}, Expected: {EXPECTED_NAME}")

        print(f"Updating target name to {EXPECTED_NAME}...")

    if bridge.set_hm10_name(EXPECTED_NAME):

        print("✅ Name updated successfully. Resetting ESP32...")

        bridge.reset()

        # Re-init after reset

        bridge = HM10ESP32Bridge(port=PORT)

    else:

        print("❌ Failed to set name. Exiting.")

        sys.exit(1)


    # 2. Connection Check

    status = bridge.get_status()

    if status != "CONNECTED":

        print(f"⚠️ ESP32 is {status}. Please ensure HM-10 is advertising. Exiting.")

        sys.exit(0)


    print(f"✨ Ready! Connected to {EXPECTED_NAME}")

    import time 

    logging.basicConfig(level=logging.INFO)
    threading.Thread(target=background_listener, args=(bridge, "diaob", "http://carcar.ntuee.org/scoreboard"), daemon=True).start()

    try:
        while True:
            cmd = input("You: ")
            if cmd.lower() in ['exit', 'quit']: break
            if cmd: bridge.send(cmd+'\n')
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()

#0.5 2