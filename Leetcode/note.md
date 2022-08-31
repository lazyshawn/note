
<!-- TOC GFM -->

* [✨ 常见算法](#-常见算法)
  - [一次遍历过程中求解二位数组的行、列最大值](#一次遍历过程中求解二位数组的行列最大值)

<!-- /TOC -->

## ✨ 常见算法
### 一次遍历过程中求解二位数组的行、列最大值
在遍历 $n\times n$ 的数组过程中，使用`grid[i][j]`和`grid[j][i]` 可以同时遍历二维
数组的行和列。
```cpp
// Problem 883: 三维形体投影面积
class Solution {
  public:
    int projectionArea(std::vector<std::vector<int>>& grid){
      // size of row/colcum
      int n = grid.size();
      int area = 0;
      for (int i=0; i<n; ++i) {
        // max value of row and colcum
        int  maxR = 0, maxC = 0;
        for (int j=0; j<n; ++j) {
          area += grid[i][j] > 0 ? 1 : 0;
          maxR = std::max(grid[i][j], maxR);
          maxC = std::max(grid[j][i], maxC);
        }
        area += (maxC + maxR);
      }
      return area;
    }
};
```

### 哈希表
> 映射函数。

根据关键码值(Key value)而直接进行访问的数据结构。

基本方法:
1. `mp.count(x)`, finds the number of element `x`.

✨ 单调栈
---------
> 通常是一维数组，要寻找任一个元素的右边或者左边第一个比自己大或者小的元素的位置，
> 此时我们就要想到可以用单调栈了。
1. 单调栈储存的元素: 存放元素的下标 $i$ 。
1. 单调栈里元素的顺序: 从栈顶到栈底，需要找大的元素则递增，反之递减。

```cpp
class Solution {
public:
  vector<int> dailyTemperatures(vector<int> &temperatures) {
    int n = temperatures.size();
    vector<int> ans(n);
    stack<int> s;
    for (int i = 0; i < n; ++i) {
      // 单调递增栈
      while (!s.empty() && temperatures[i] > temperatures[s.top()]) {
        int previousIndex = s.top();
        ans[previousIndex] = i - previousIndex;
        s.pop();
      }
      s.push(i);
    }
    return ans;
  }
};
```


## ✨ 线段树

## ✨ 并查集

✨ 动态规划
-----------
### 基本定义
动态规划，英文： Dynamic Programming ，简称 DP 。动态规划中每一个状态一定是由前几个
状态推导出来的。如果某一问题有很多重叠子问题，使用动态规划是最有效的。

$dp[i] = f(dp[i-1], dp[i-2], \cdots )$

1. 状态转移方程。
1. 边界条件。

求解动态规划问题的基本步骤:
1. 确定 dp 数组及其下标的含义。 dp 数组往往表示遍历过程中子问题的解。 dp 的第一
   个下标为子问题的序号。
1. 确定递推公式。
1. dp 数组如何初始化。
1. 确定遍历顺序。
1. 举例推导 dp 数组。

动态规划中的小 trick:
1. 如果当前状态只由前几个状态决定，可以使用「滚动数组思想/滑动窗口思想」优化空间
   复杂度。每次遍历和计算状态值时维护前一个下标处的状态值即可。

### 背包问题
1. 01 背包;
1. 完全背包;


## ✨ 深度优先算法 & 广度优先算法
> Ref: [知乎 - 七十九、深度和广度优先搜索算法](
https://zhuanlan.zhihu.com/p/338416868)
### 深度优先算法
深度优先算法的本质是回溯算法，多数是应用在树上，一个比较典型的应用就是二叉树的中
序遍历。

DFS 的实现考虑要以下几个问题即可：
1. **边界范围** ：「即搜索终止条件，递归结束条件。」
1. **可供选择的范围列表** ：「所有可供选择的范围列表。」
1. **已做出的选择列表** ：「标记当前已经做出的选择。」

深度优先搜索是图论中的经典算法，利用深度优先搜索算法可以产生目标图的相应拓扑排序
表，利用拓扑排序表可以方便的解决很多相关的图论问题，如最大路径问题等等。一般用堆
数据结构来辅助实现 DFS 算法。根据深度优先搜索的特点，采用递归函数实现比较简单。

```cpp
// 节点遍历的方向
static const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
std::vector<std::vector<int>> heights;

void dfs(int row, int col, std::vector<std::vector<bool>> &ocean) {
  int m = ocean.size();
  int n = ocean[0].size();
  // 边界范围: 即搜索终止条件，递归结束条件
  if (ocean[row][col]) {
    return;
  }
  // 标记当前作出的选择
  ocean[row][col] = true;
  for (int i = 0; i < 4; i++) {
    int newRow = row + dirs[i][0], newCol = col + dirs[i][1];
    // 可供选择的范围列表
    if (newRow >= 0 && newRow < m && newCol >= 0 && newCol < n &&
        heights[newRow][newCol] >= heights[row][col]) {
      dfs(newRow, newCol, ocean);
    }
  }
}
```

### 广度优先算法
先访问完当前顶点的所有邻接点，然后再访问下一层的所有节点，该算法适用于解决最短、
最小路径等问题，但是构建广度优先算法需要维护自己的队列。

比如，二叉树的层次遍历，我们大概会有如下几个步骤：
1. 向 Queue 中放入起始节点。
1. 只要这个 Queue 中有元素就一直遍历。
1. 每次遍历时，首先计算一下当前 Queue 里有多少个元素，这就是这棵树当前层级元素的数
   量，记作 Size 。
1. 接下来从 Queue 中移除 Size 中的多个元素，对他们进行符合我们题目的一些操作。
1. 移除每个节点时，把它们的子节点添加进 Queue 中。
1. 只要 Queue 中还有元素，说明还有下一个层级，就一直重复步骤 3 去处理下一个层级。

```cpp
void bfs(int row, int col, std::vector<std::vector<bool>> &ocean) {
  if (ocean[row][col]) {
    return;
  }
  int m = heights.size();
  int n = heights[0].size();
  ocean[row][col] = true;
  // 储存节点的队列
  std::queue<std::pair<int, int>> qu;
  // 起始节点
  qu.emplace(row, col);
  // 搜索队列中的节点
  while (!qu.empty()) {
    // 移除队列中的元素
    auto [row, col] = qu.front();
    qu.pop();
    for (int i = 0; i < 4; i++) {
      int newRow = row + dirs[i][0], newCol = col + dirs[i][1];
      // !ocean[newRow][newCol]: 避免重复搜索
      if (newRow >= 0 && newRow < m && newCol >= 0 && newCol < n &&
          heights[newRow][newCol] >= heights[row][col] &&
          !ocean[newRow][newCol]) {
        ocean[newRow][newCol] = true;
        // 遍历一个节点后将其子节点加入队列
        qu.emplace(newRow, newCol);
      }
    }
  }
}
```

### Dijkstra 算法
> 求一个图中一个点到其他所有点的最短路径

输入: 邻接矩阵图;  
维护: 2 个数组 --- `dist` 已访问的点到起点的最短距离; `used` 各个点的访问状态。  
算法:
1. 初始化维护的数组。`dist` 中起点值为0，其余值为inf; `used` 值全为 `0/false` ;
1. 从 `used` 中取出到起点路径最短的点，放入 `result` 中，并更新 `used` ;
1. 动态规划，更新 `dist` 中各个点的值;
1. 重复 2~3 直至 `used` 全为 `true`.

```cpp
std::vector<int> Solution::dijkstra(std::vector<std::vector<int>> graph, int startIndex) {
  // 初始化
  const int n = graph.size(), inf = INT_MAX/2;
  std::vector<int> dist(n,inf), used(n);
  dist[startIndex] = 0;

  for (int i=0; i<n; ++i) {
    // 找到未访问的最小值
    int index = -1;
    for (int j=0; j<n; ++j) {
      if (!used[j] && (index+1==0 || dist[j] < dist[index])) {
        index = j;
      }
    }
    used[index] = true;
    // 更新到起点距离
    for (int i=0; i<n; ++i) {
      dist[i] = min(dist[i], dist[index]+graph[index][i]);
    }
  }
  return dist;
}
```

