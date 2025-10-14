## =========================================================================
## @author Leonardo Florez-Valencia (florez-l@javeriana.edu.co)
## =========================================================================

import sys, heapq, math
from collections import deque
from MeshViewer import *

'''
'''
def distance( a, b ):
   return sum( [ ( a[ i ] - b[ i ] ) ** 2 for i in range( len( a ) ) ] ) ** 0.5
# end def

'''
'''
def DijkstraCheapest(G, start, end):
    V, A = G
    n = len(V)
    dist = [math.inf]*n
    prev = [-1]*n
    dist[start] = 0
    Q = [(0, start)]

    while len( Q ) > 0:
        cost, u = heapq.heappop(Q)
        if u == end:
            break
        for v in A[u]:
            d = distance(V[u], V[v])
            if dist[u] + d < dist[v]:
                dist[v] = dist[u] + d
                prev[v] = u
                heapq.heappush(Q, (dist[v], v))

    path = []
    u = end
    if prev[u] == -1:
        return []
    while u != -1:
        path.insert(0, u)
        u = prev[u]
    return path
# end def

'''
'''
def DijkstraShortest(G, start, end):
    V, A = G
    n = len(V)
    dist = [math.inf] * n
    prev = [-1] * n

    dist[start] = 0
    q = deque([start])

    while q:
        u = q.popleft()
        if u == end:
            break
        for v in A[u]:
            if dist[v] is math.inf:
                dist[v] = dist[u] + 1
                prev[v] = u
                q.append(v) # Encolar no visitados

    if dist[end] is math.inf:
        return []

    path = []
    u = end
    while u != -1:
        path.insert(0, u)
        u = prev[u]
    return path
# end def

'''
'''
def KruskalShortest( G, start, end ):
  V, A = G
  
  n = len( V )
  marks = [ False for i in range( n ) ]
  Tree = [i for i in range( n )]
  minQueue = [ ( float( 0 ), start, start ) ]
  
  while len(minQueue) > 0:
    ( c, i, p ) = heapq.heappop( minQueue )
    
    if not marks[i]:
      marks[ i ] = True
      Tree[ i ] = p

      for j in A[i]:
          heapq.heappush( minQueue, ( 1 + c, j, i ) )
  
  Final = SpanningTree_Backtrack( Tree, start, end )
  return Final
# end def

'''
'''
def SpanningTree_Backtrack(T, s, e):
    if s == e:
        return [s]
    #end if

    n = len(T)
    if not (0 <= s < n and 0 <= e < n):
        return []          # índices fuera de rango
    #end if

    if T[e] == e:
        return []          # nunca se le asignó padre distinto a e
    #end if

    path = []
    seen = set()
    j = e
    while j != s:
        if j in seen:
            return []      # ciclo en la tabla de padres
        #end if
        seen.add(j)

        parent = T[j]
        if parent == j or parent == -1:
            return []      # padre sin asignar o marcador inválido
        #end if

        path.insert(0, j)
        j = parent
    #end while

    path.insert(0, s)
    return path
# end def
'''
'''
def KruskalCheapest( G, start, end ):
  V, A = G
  
  n = len( V )
  marks = [ False for i in range( n ) ]
  Tree = [i for i in range( n )]
  minQueue = [ ( float( 0 ), start, start ) ]
  
  while len(minQueue) > 0:
    ( c, i, p ) = heapq.heappop( minQueue )
    
    if not marks[i]:
      marks[ i ] = True
      Tree[ i ] = p

      for j in A[i]:
          heapq.heappush( minQueue, ( distance( V[ i ], V[ j ] ), j, i ) )
  
  Final = SpanningTree_Backtrack( Tree, start, end )
  return Final
# end def



'''
'''
def main( argv ):
  fname = argv[ 1 ]
  pId = int( argv[ 2 ] )

  # Read data
  viewer = MeshViewer( fname, opacity = 0.8 )
  V, A = viewer.graph( )
  print(V)

  # Correct pId
  pId = pId % len( V )

  # Get farthest point
  qId = pId
  fDist = 0
  for i in range( len( V ) ):
    d = distance( V[ pId ], V[ i ] )
    if fDist < d:
      fDist = d
      qId = i
    # end if
  # end for

  # Get paths
  P  = [ DijkstraShortest( ( V, A ), pId, qId ) ]
  P += [ DijkstraCheapest( ( V, A ), pId, qId ) ]
  P += [ KruskalShortest( ( V, A ), pId, qId ) ]
  P += [ KruskalCheapest( ( V, A ), pId, qId ) ]

  # Define colors
  # R Dijkstra Caminos cortos
  # G Dijkstra Caminos baratos
  # B Kruskal Caminos cortos
  # Black Kruskal Caminos baratos
  C = [ ( 1, 0, 0 ), ( 0, 1, 0 ), ( 0, 0, 1 ), ( 0, 0, 0 ) ]
  for i in range( 4 ) :
    viewer.add_path( P[ i ], C[ i ] )
  # end for

  viewer.show( )
# end def

""" ==================================================================== """
if __name__ == '__main__':
  if len( sys.argv ) < 3:
    print(
        'Usage: ' + sys.argv[ 0 ] + ' file.obj pId',
        file = sys.stderr, flush = True
        )
    sys.exit( 1 )
  # end if
  main( sys.argv )
# end if

## eof - compare_paths.py
