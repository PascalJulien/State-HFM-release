# CMakeLists.txt for hubomz/mzcommon

set(mzcommon_srcs
  mersenne.c
  gauss.c
  GlCamera.cpp
  GlFrameGrabber.cpp
  GlTexture2D.cpp
  MzGlutApp.cpp
  TinyDom.cpp
  TriMesh3.cpp
  HeightMap.cpp
  DtGrid.cpp
  Geom2.cpp
  glstuff.cpp
  CCDWrapper.cpp)

if (GLEW_FOUND)
  set(mzcommon_srcs ${mzcommon_srcs} GlFramebufferObject.cpp)
endif()

add_library( mzcommon ${mzcommon_srcs} )
target_link_libraries(mzcommon ${OPENGL_LIBRARY} ${GLUT_LIBRARY} ${EXPAT_LIBRARY} ${PNG_LIBRARY} ${CCD_LDFLAGS} )
