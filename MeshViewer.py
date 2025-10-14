## =========================================================================
## @author Leonardo Florez-Valencia (florez-l@javeriana.edu.co)
## =========================================================================

import vtk

"""
"""
class MeshViewer:

  '''
  '''
  S = []
  G = ( None, None )
  M = []
  A = []

  '''
  '''
  def __init__( self, fname, rgb = ( 1, 1, 1 ), opacity = 0.5 ):

    self._read( fname )

    # Prepare graph
    N = self.S[ 0 ].GetNumberOfPoints( )
    C = self.S[ 0 ].GetNumberOfCells( )
    V = [ self.S[ 0 ].GetPoint( n ) for n in range( N ) ]
    A = [ set( [] ) for n in range( N ) ]

    # Fill adjacency structure and set graph tuple
    idl = vtk.vtkIdList( )
    for c in range( C ):
      self.S[ 0 ].GetCellPoints( c, idl )
      idx = [ idl.GetId( i ) for i in range( idl.GetNumberOfIds( ) ) ]
      for i in range( len( idx ) ):
        A[ idx[ i ] ].add( idx[ ( i + 1 ) % len( idx ) ] )
      # end for
    # end for
    self.G = ( V, A )

    # Prepare visualization objects
    mapper = vtk.vtkPolyDataMapper( )
    mapper.SetInputData( self.S[ 0 ] )
    actor = vtk.vtkActor( )
    actor.SetMapper( mapper )
    actor.GetProperty( ).SetColor( rgb )
    actor.GetProperty( ).SetOpacity( opacity )
    self.M = [ mapper ]
    self.A = [ actor ]
  # end def

  '''
  '''
  def graph( self ):
    return self.G
  # end def

  '''
  '''
  def add_path( self, P, rgb, w = 2 ):
    points = vtk.vtkPoints( )
    line = vtk.vtkCellArray( )
    line.InsertNextCell( len( P ) )
    for i in range( len( P ) ):
      points.InsertNextPoint( self.S[ 0 ].GetPoint( P[ i ] ) )
      line.InsertCellPoint( i )
    # end for

    self.S += [ vtk.vtkPolyData( ) ]
    self.S[ -1 ].SetPoints( points )
    self.S[ -1 ].SetLines( line )

    self.M += [ vtk.vtkPolyDataMapper( ) ]
    self.M[ -1 ].SetInputData( self.S[ -1 ] )
    self.A += [ vtk.vtkActor( ) ]
    self.A[ -1 ].SetMapper( self.M[ -1 ] )
    self.A[ -1 ].GetProperty( ).SetColor( rgb[ 0 ], rgb[ 1 ], rgb[ 2 ] )
    self.A[ -1 ].GetProperty( ).SetLineWidth( w )
  # end def

  '''
  '''
  def show( self ):

    ren = vtk.vtkRenderer( )
    ren.SetBackground( 0.8, 0.8, 0.2 )
    for a in self.A:
      ren.AddActor( a )
    # end for

    win = vtk.vtkRenderWindow( )
    win.AddRenderer( ren )
    win.SetSize( 800, 600 )

    rwi = vtk.vtkRenderWindowInteractor( )
    rwi.SetRenderWindow( win )

    sty = vtk.vtkInteractorStyleTrackballCamera( )
    rwi.SetInteractorStyle( sty )
    
    win.Render( )
    rwi.Initialize( )
    rwi.Render( )
    rwi.Start( )
  # end def

  '''
  '''
  def _read( self, fname ):
    # Read OBJ
    r = vtk.vtkOBJReader( )
    r.SetFileName( fname )
    r.Update( )

    # Deep copy vtkPolyData without its associated pipeline
    self.S = [ vtk.vtkPolyData( ) ]
    self.S[ 0 ].DeepCopy( r.GetOutput( ) )

    # Prepare remaining data
    self.G = ( None, None )
  # end def

# end class

## eof - MeshViewer.py
