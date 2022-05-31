
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

## ✨ 线段树

## ✨ 并查集

## ✨ 动态规划

## ✨ 深度优先算法 & 广度优先算法
> Ref: [知乎 - 七十九、深度和广度优先搜索算法](
https://zhuanlan.zhihu.com/p/338416868)
### 深度优先算法
深度优先算法的本质是回溯算法，多数是应用在树上，一个比较典型的应用就是二叉树的中
序遍历。

DFS的实现考虑要以下几个问题即可：
1. **边界范围** ：「即搜索终止条件，递归结束条件。」
1. **可供选择的范围列表** ：「所有可供选择的范围列表。」
1. **已做出的选择列表** ：「标记当前已经做出的选择。」

深度优先搜索是图论中的经典算法，利用深度优先搜索算法可以产生目标图的相应拓扑排序
表，利用拓扑排序表可以方便的解决很多相关的图论问题，如最大路径问题等等。一般用堆
数据结构来辅助实现DFS算法。根据深度优先搜索的特点，采用递归函数实现比较简单。

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
1. 向Queue中放入起始节点。
1. 只要这个Queue中有元素就一直遍历。
1. 每次遍历时，首先计算一下当前Queue里有多少个元素，这就是这棵树当前层级元素的数
   量，记作Size。
1. 接下来从Queue中移除Size中的多个元素，对他们进行符合我们题目的一些操作。
1. 移除每个节点时，把它们的子节点添加进Queue中。
1. 只要Queue中还有元素，说明还有下一个层级，就一直重复步骤3去处理下一个层级。

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

