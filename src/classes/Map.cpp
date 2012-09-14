#include "Map.h"


Map::Map() {
}

Map::~Map() {
}

void Map::drawMap() {
  float fQuadHeight = 0.65;
  float fQuadWidth = fQuadHeight;
  float fXOffset = -0.89;
  float fYOffset = 0.575;
  
  // NOTE: 2D textures are disabled here at the moment. When not
  // disabling, the current camera frame texture is applied to the
  // square, making it practically invisible. As long as there is no
  // actual map available to display here, the textures will stay
  // disabled and the square is shown in plain white.
  
  glLoadIdentity();
  glDisable(GL_TEXTURE_2D);
  glTranslatef(0, 0, -2.5);
  glBegin(GL_QUADS);
  {
    glColor3f(1, 1, 1);
    
    glVertex2f(fXOffset + fQuadWidth / 2, fYOffset + fQuadHeight / 2);
    //    glTexCoord2d(0, 1);
    glVertex2f(fXOffset + fQuadWidth / 2, fYOffset - fQuadHeight / 2);
    //    glTexCoord2d(1, 1);
    glVertex2f(fXOffset - fQuadWidth / 2, fYOffset - fQuadHeight / 2);
    //    glTexCoord2d(1, 0);
    glVertex2f(fXOffset - fQuadWidth / 2, fYOffset + fQuadHeight / 2);
    //    glTexCoord2d(0, 0);
  }
  glEnd();
  glEnable(GL_TEXTURE_2D);
}
