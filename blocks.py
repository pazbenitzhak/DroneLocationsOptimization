from collections import deque

class block:
    def classifyRouteBlocks(arr):
        # gets a matrix which consists solely of 0,1 values
        rows = len(arr)
        cols = len(arr[0])
        spots = []
        visited = [[False for num in range(cols)] for num in range(rows)]
        for i in range(rows):
            for j in range(cols):
                if arr[i][j] == 1 and not visited[i][j]:
                    # Found a new spot
                    spot = []
                    size = 0
                    q = deque([(i, j)])
                    visited[i][j] = True
                    while q:
                        r, c = q.popleft()
                        spot.append((r, c))
                        size += 1
                        for dr, dc in ((1,0), (-1,0), (0,1), (0,-1), (1,-1), (1,1), (-1,-1), (-1,1)):
                            if 0 <= r + dr < rows and 0 <= c + dc < cols and arr[r+dr][c+dc] == 1 and not visited[r+dr][c+dc]:
                                q.append((r+dr, c+dc))
                                visited[r+dr][c+dc] = True
                    # Classify the spot by its indices and size
                    if size>1000:
                        spots.append(spot)
        return spots

