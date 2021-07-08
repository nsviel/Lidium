#include "FrameBuffer.h"

//Constructor / Destructor
FrameBuffer::FrameBuffer() : width(1024), height(768){
  /*create_Texture();
  create_FBO();
  create_DepthRenderBuffer();
  create_bckgrndQuad();*/
}
FrameBuffer::~FrameBuffer(){
}

//FBO functions
void FrameBuffer::display(){
  // -------------------------------------------------------------------------------------------------------------
  // 1ere passe OpenGL (model -> texture)
  // -------------------------------------------------------------------------------------------------------------

  // Activation du test de profondeur
  glEnable(GL_DEPTH_TEST);

  // Activation et binding la texture
  glBindTexture(GL_TEXTURE_2D, idTexture);
  glActiveTexture(GL_TEXTURE0);

  // Activation du FBO
  glBindFramebuffer(GL_FRAMEBUFFER, idFBO);
  glViewport(0, 0, 1024, 768);

  // Changement de la couleur de background
  glClearColor(0.0f,0.0f,0.0f,1.0f);

  // Rafraichissement des buffers (reset)
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  //LoadShaders(); // Chargement des shaders
  //LoadUniform(); // Transmission des variables uniformes au shaders
  //model->draw(); // Rendu du modèle dans la texture

  cout<<"First pass OK !"<<endl;

  // -------------------------------------------------------------------------------------------------------------
  // Deuxième passe OpenGL (texture -> quad)
  // -------------------------------------------------------------------------------------------------------------

  // Désactivation du FBO
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glViewport(0, 0, width, height);

  // Desactivation du test de profondeur
  glDisable(GL_DEPTH_TEST);

  //LoadShaders(); // Chargement des shaders
  //LoadUniform(); // Transmission des variables uniformes au shaders
  //quad->draw(); // Rendu du quad

  // Désactivation de la texture
  glBindTexture(GL_TEXTURE_2D, 0);

  cout<<"Second pass OK !"<<endl;
}
void FrameBuffer::create_FBO(){
  // Génération d'un second FBO
  idFBO = 0;
  glGenFramebuffers(1, &idFBO);

  // On bind le FBO
  glBindFramebuffer(GL_FRAMEBUFFER, idFBO);

  // Affectation de notre texture au FBO
  glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, idTexture, 0);

  // Affectation d'un drawbuffer au FBO
  GLenum DrawBuffers[2] = {GL_COLOR_ATTACHMENT0};
  glDrawBuffers(1, DrawBuffers);

  // On débind le FBO
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  cout<<"FBO created ..."<<endl;
}
void FrameBuffer::create_Texture(){
  // Génération d'une texture
  glGenTextures(1, &idTexture);

  // Binding de la texture pour pouvoir la modifier.
  glBindTexture(GL_TEXTURE_2D, idTexture);

  // Création de la texture 2D vierge de la taille de votre fenêtre OpenGL
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1024, 768, 0, GL_RGB, GL_UNSIGNED_BYTE, 0);

  // Paramètrage de notre texture (étirement et filtrage)
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

  cout<<"Texture created ..."<<endl;
}
void FrameBuffer::create_DepthRenderBuffer(){
  // Création d'un buffer de profondeur
  glGenRenderbuffers(1, &idDepthrenderbuffer);

  // Binding du buffer de profondeur
  glBindRenderbuffer(GL_RENDERBUFFER, idDepthrenderbuffer);

  // Paramètrage du RenderBuffer (selon la taille du canvas OpenGL)
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, 1024, 768);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, idDepthrenderbuffer);

  cout<<"Depth Render Buffer created ..."<<endl;
}
void FrameBuffer::create_bckgrndQuad(){
    int numberOfVertices =  4;
    int numberOfTriangles = 2;

    //Initialisation du tableau contenant les coordonnées XYZ des sommets dans le monde
    GLfloat* positions = new GLfloat[3 * numberOfVertices];

    //Initialisation du tableau contenant les coordonnées XYZ des sommets dans la texture
    GLfloat* texCoords = new GLfloat[2 * numberOfVertices];

    //Initialisation du tableau contenant indices des triangles
    GLfloat* indexes = new GLfloat[3 * numberOfTriangles];

    // Coin bas-gauche
    positions[0] = -1.0f;
    positions[1] = -1.0f;
    positions[2] = 0.0f;
    texCoords[0] = 0.0f;
    texCoords[1] = 0.0f;

    // Coin haut-gauche
    positions[3] = -1.0f;
    positions[4] = 1.0f;
    positions[5] = 0.0f;
    texCoords[2] = 0.0f;
    texCoords[3] = 1.0f;

    // Coin haut-droit
    positions[6] = 1.0f;
    positions[7] = 1.0f;
    positions[8] = 0.0f;
    texCoords[4] = 1.0f;
    texCoords[5] = 1.0f;

    // Coin bas-droit
    positions[9] = 1.0f;
    positions[10] = -1.0f;
    positions[11] = 0.0f;
    texCoords[6] = 1.0f;
    texCoords[7] = 0.0f;

    // Face triangulaire 1
    indexes[0] = 0;
    indexes[1] = 1;
    indexes[2] = 2;

    // Face triangulaire 2
    indexes[3] = 2;
    indexes[4] = 3;
    indexes[5] = 0;

    //Génération des VBO
    GLuint idOfPositionArray, idOfTexCoordArray, idOfIndexArray;
    glGenBuffers(1, &idOfPositionArray);
    glGenBuffers(1, &idOfTexCoordArray);
    glGenBuffers(1, &idOfIndexArray);

    //Remplissage des VBO
    glBindBuffer(GL_ARRAY_BUFFER, idOfPositionArray);
    glBufferData(GL_ARRAY_BUFFER, 3 * numberOfVertices * sizeof(GLfloat), positions, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, idOfTexCoordArray);
    glBufferData(GL_ARRAY_BUFFER,  2 * numberOfVertices * sizeof(GLfloat), texCoords, GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, idOfIndexArray);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, 3 * numberOfTriangles * sizeof(GLuint), indexes, GL_STATIC_DRAW);

    cout<<"Background quad created ..."<<endl;
}

void FrameBuffer::readScreenPixels(int width, int height){
  //--------> Methode 1 : tres lente
  /*cout<<"Make screenshot ... "<<width<<" "<<height<<endl;

  float pixel[4];
  pngwriter png(width,height,0,"ScreenShot.png");

  for(int i=0; i<width; i++)
    for(int j=0; j<height; j++)
    {
      glReadPixels(i, j, 1, 1, GL_RGBA, GL_FLOAT, &pixel);
      png.plot(i,j, pixel[0], pixel[1], pixel[2]);
    }

  png.close();
  cout<<"Screenshot Ok"<<endl;*/

  //--------> Methode 1 : tres rapide
  cout<<"Make screenshot ... "<<width<<" "<<height<<endl;

  int nPixels= 3 * width * height;
  GLfloat* pixels = new GLfloat[nPixels];
  glReadPixels(0.0, 0.0, width, height, GL_RGB, GL_FLOAT, pixels);

  //Requiere the external PGGwriter librairy - must also be included in the CMakelists file
  /*
  pngwriter PNG(width, height, 0.0, "ScreenShot.png");
  size_t x = 0;
  size_t y = 0;
  float R, G, B;
  int cpt = 0;
  for(size_t i=0; i<nPixels/3; i++)
  {
    PNG.plot(x, y, pixels[cpt], pixels[cpt+1], pixels[cpt+2]);
    x++;
    cpt += 3;
    if(x == width)
    {
      x = 0;
      y++;
    }
  }
  PNG.close();
  */

  cout<<"Screenshot ok "<<endl;
}
