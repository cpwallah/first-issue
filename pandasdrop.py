Remove rows or columns by specifying label names and corresponding axis, or by directly specifying index or column names. When using a multi-index, labels on different levels can be removed by specifying the level. See the user guide for more information about the now unused levels.

Parameters
:
labels
single label or list-like
Index or column labels to drop. A tuple will be used as a single label and not treated as a list-like.

axis
{0 or ‘index’, 1 or ‘columns’}, default 0
Whether to drop labels from the index (0 or ‘index’) or columns (1 or ‘columns’).

index
single label or list-like
Alternative to specifying axis (labels, axis=0 is equivalent to index=labels).

columns
single label or list-like
Alternative to specifying axis (labels, axis=1 is equivalent to columns=labels).

level
int or level name, optional
For MultiIndex, level from which the labels will be removed.

inplace
bool, default False
If False, return a copy. Otherwise, do operation in place and return None.

errors
{‘ignore’, ‘raise’}, default ‘raise’
If ‘ignore’, suppress error and only existing labels are dropped.

  #RESHAPE
  df = pd.DataFrame(np.arange(12).reshape(3, 4),
                  columns=['A', 'B', 'C', 'D'])
df
   A  B   C   D
0  0  1   2   3
1  4  5   6   7
2  8  9  10  11
