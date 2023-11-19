// *********************************************************************
// **
// ** Asignatura: INFORMÁTICA GRÁFICA
// ** 
// ** Mallas indexadas (implementación)
// ** Copyright (C) 2016-2023 Carlos Ureña
// **
// ** Implementación de las clases 
// **        + MallaInd: malla indexada de triángulos (derivada de Objeto3D)
// **        + MallaPLY: malla indexada de triángulos, leída de un PLY (derivada de MallaInd)
// **        + algunas clases derivadas de MallaInd.
// **
// ** This program is free software: you can redistribute it and/or modify
// ** it under the terms of the GNU General Public License as published by
// ** the Free Software Foundation, either version 3 of the License, or
// ** (at your option) any later version.
// **
// ** This program is distributed in the hope that it will be useful,
// ** but WITHOUT ANY WARRANTY; without even the implied warranty of
// ** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// ** GNU General Public License for more details.
// **
// ** You should have received a copy of the GNU General Public License
// ** along with this program.  If not, see <http://www.gnu.org/licenses/>.
// **
// *********************************************************************


#include "ig-aux.h"
#include "aplicacion-ig.h"
#include "malla-ind.h"   // declaración de 'ContextoVis'
#include "lector-ply.h"
#include "seleccion.h"   // para 'ColorDesdeIdent' 


// *****************************************************************************
// funciones auxiliares

// *****************************************************************************
// métodos de la clase MallaInd.

MallaInd::MallaInd()
{
   // nombre por defecto
   ponerNombre("malla indexada, anónima");
}
// -----------------------------------------------------------------------------

MallaInd::MallaInd( const std::string & nombreIni )
{
   // 'identificador' puesto a 0 por defecto, 'centro_oc' puesto a (0,0,0)
   ponerNombre(nombreIni) ;
}

//-----------------------------------------------------------------------------
// calcula la tabla de normales de triángulos una sola vez, si no estaba calculada

void MallaInd::calcularNormalesTriangulos()
{

   
   // si ya está creada la tabla de normales de triángulos, no es necesario volver a crearla
   const unsigned nt = triangulos.size() ;
   assert( 1 <= nt );
   if ( 0 < nor_tri.size() )
   {
      assert( nt == nor_tri.size() );
      return ;
   }

   // COMPLETAR: Práctica 4: creación de la tabla de normales de triángulos
   // ....

}


// -----------------------------------------------------------------------------
// calcula las dos tablas de normales

void MallaInd::calcularNormales()
{
   using namespace glm ;
   // COMPLETAR: en la práctica 4: calculo de las normales de la malla
   // se debe invocar en primer lugar 'calcularNormalesTriangulos'
   // .......


}


// --------------------------------------------------------------------------------------------

void MallaInd::visualizarGL( )
{
   // comprobar algunas precondiciones básicas
   using namespace std ;
   assert( apl != nullptr );
   Cauce * cauce = apl->cauce ; assert( cauce != nullptr );
   CError();

   // si la malla no vértices o no tiene triángulos, imprimir advertencia y salir.
   if ( triangulos.size() == 0 || vertices.size() == 0 )
   {  cout << "advertencia: intentando dibujar malla vacía '" << leerNombre() << "'" << endl << flush ;
      return ;
   }

   // COMPLETAR: práctica 1: cambiar color del cauce
   //
   // Si el objeto tiene un color asignado (se comprueba con 'tieneColor')
   //    - hacer push del color actual del cauce
   //    - fijar el color en el cauce usando el color del objeto (se lee con 'leerColor()')
   if(tieneColor()){
      cauce->pushColor();
      cauce->fijarColor(leerColor());
   }

   // COMPLETAR: práctica 1: crear el descriptor de VAO, si no está creado
   //  Si el puntero 'dvao' es nulo, crear el descriptor de VAO
   //   * en primer lugar se crea el descriptor de VAO, con su constructor 
   //     (se le pasa como parámetro la tabla de posiciones y el número de atributos del cauce). 
   //   * se añade el descriptor de VBO con la tabla de índices (la tabla de triángulos),
   //   * finalmente se añaden al VAO los descriptores VBOs con tablas de atributos 
   //     que no estén vacías
   //  Si el VAO ya está creado, (dvao no nulo), no hay que hacer nada.
   //
   if(!dvao){
      DescrVBOAtribs* puntero_vbo;
      //creamos un puntero a un VBO que almacene las posiciones de los vertices
      puntero_vbo=new DescrVBOAtribs(ind_atrib_posiciones,vertices);
     //creamos el VAO
     dvao=new DescrVAO(numero_atributos_cauce,puntero_vbo);

     //ahora asociamos al VAO de la malla el VBO con los indices
     DescrVBOInds* indices_vertices=new DescrVBOInds(triangulos);
     dvao->agregar(indices_vertices);

     //comprobamos ahora si la malla contiene tb info sobre el resto de atributos
     //¿contiene informacion sobre los colores de los vertices?
     if(!col_ver.empty()){
      puntero_vbo=new DescrVBOAtribs(ind_atrib_colores,col_ver);
      dvao->agregar(puntero_vbo);
     }
     if(!nor_ver.empty()){
      puntero_vbo=new DescrVBOAtribs(ind_atrib_normales,nor_ver);
      dvao->agregar(puntero_vbo);
     }
     if(!cc_tt_ver.empty()){
      puntero_vbo=new DescrVBOAtribs(ind_atrib_coord_text,cc_tt_ver);
      dvao->agregar(puntero_vbo);
     }

   }


   // COMPLETAR: práctica 1: visualizar el VAO usando el método 'draw' de 'DescrVAO'
   dvao->draw(GL_TRIANGLES);

   // COMPLETAR: práctica 1: restaurar color anterior del cauce 
   //
   // Si el objeto tiene un color asignado (se comprueba con 'tieneColor')
   //    - hacer 'pop' del color actual del cauce
   if(tieneColor()){
      cauce->popColor();
   }

}


// -----------------------------------------------------------------------------
// Visualizar el objeto con OpenGL
// usa las tablas de normales, colores y coordenadas de textura, si no están vacías.
      
void MallaInd::visualizarGeomGL( )
{
   // Comprobar que el descriptor de VAO ya está creado
   // (es decir, este método únicamente se podrá invocar después de que 
   // se haya llamado a 'visualizaGL')
   
   assert( dvao != nullptr );

   // COMPLETAR: práctica 1: visualizar únicamente la geometría del objeto 
   // 
   //    1. Desactivar todas las tablas de atributos del VAO (que no estén vacías)
   //    2. Dibujar la malla (únicamente visualizará los triángulos)
   //    3. Volver a activar todos los atributos para los cuales la tabla no esté vacía
   // ....

   if(!col_ver.empty()){
      dvao->habilitarAtrib(ind_atrib_colores,false);
   }
   if(!nor_ver.empty()){
      dvao->habilitarAtrib(ind_atrib_normales,false);
   }
   if(!cc_tt_ver.empty()){
      dvao->habilitarAtrib(ind_atrib_coord_text,false);
   }

   dvao->draw(GL_TRIANGLES);

   //volvemos a habilitar los atributos desabilitados
   if(!col_ver.empty()){
      dvao->habilitarAtrib(ind_atrib_colores,true);
   }
   if(!nor_ver.empty()){
      dvao->habilitarAtrib(ind_atrib_normales,true);
   }
   if(!cc_tt_ver.empty()){
      dvao->habilitarAtrib(ind_atrib_coord_text,true);
   }



}

// -----------------------------------------------------------------------------
// Visualizar las normales del objeto, si no tiene tabla de normales imprime 
// advertencia y no hace nada.

void MallaInd::visualizarNormalesGL(  )
{
   using namespace std ;
   assert( apl != nullptr );
   Cauce * cauce = apl->cauce ; assert( cauce != nullptr );

   if ( nor_ver.size() == 0 )
   {
      cout << "Advertencia: intentando dibujar normales de una malla que no tiene tabla (" << leerNombre() << ")." << endl ;
      return ;
   }  

   if( nor_ver.size() != vertices.size() )
   {
      cout << "Error visu. normales: tabla de normales no vacía y de tamaño distinto a la de vértices." << endl ;
      cout << "Nombre del objeto        : " << leerNombre() << endl ;
      cout << "Tamaño tabla vértices    : " << vertices.size() << endl ;
      cout << "Tamaño tabla de normales : " << nor_ver.size() << endl ;
      exit(1);
   }
   CError();

   // COMPLETAR: práctica 4: visualizar las normales del objeto MallaInd
   // 
   // *1* Si el puntero al descriptor de VAO de normales ('dvao_normales') es nulo, 
   //    debemos de crear dicho descriptor, con estos pasos:
   //
   //       * Para cada posición 'v_i' de un vértice en el vector 'vertices':
   //             - Leer la correspondiente normal 'n_i' del vector de normales ('nor_ver').
   //             - Añadir 'v_i' al vector 'segmentos_normales'.
   //             - Añadir 'v_i+a*n_i' al vector 'segmentos_normales'.
   //
   //       * Crear el objeto descriptor del VAO de normales, para ello se usa el vector 
   //          'segmentos_normales' y se tiene en cuenta que esa descriptor únicamente gestiona 
   //          una tabla de atributos de vértices (la de posiciones, ya que las otras no se 
   //          necesitan).
   // 
   // *2* Visualizar el VAO de normales, usando el método 'draw' del descriptor, con el 
   //       tipo de primitiva 'GL_LINES'.

   //  ..........

}

// -----------------------------------------------------------------------------
// visualizar el objeto en 'modo seleccion', es decir, sin iluminación y con los colores 
// basados en los identificadores de los objetos
void MallaInd::visualizarModoSeleccionGL() 
{

   using namespace std ;
   assert( apl != nullptr );
   Cauce * cauce = apl->cauce ; assert( cauce != nullptr );

   // COMPLETAR: práctica 5: visualizar la malla en modo selección 
   //
   // Se debe escribir código para visualizar únicamente la geometría, pero usando el color 
   // obtenido a partir del identificador. El código da estos pasos:
   // 
   // 1. Leer el identificador del objeto (con 'leerIdentificador'). Si el objeto tiene 
   //    identificador (es decir, si su identificador no es -1)
   //       + Hacer push del color del cauce, con 'pushColor'.
   //       + Fijar el color del cauce (con 'fijarColor') usando un color obtenido a 
   //         partir del identificador (con 'ColorDesdeIdent'). 
   // 2. Invocar 'visualizarGeomGL' para visualizar la geometría.
   // 3. Si tiene identificador: hacer pop del color, con 'popColor'.
   //

}


// ****************************************************************************
// Clase 'MallaPLY'

MallaPLY::MallaPLY( const std::string & nombre_arch )
{
   ponerNombre( std::string("malla leída del archivo '") + nombre_arch + "'" );

   // COMPLETAR: práctica 2: leer archivo PLY e inicializar la malla
   // ..........................
   LeerPLY(nombre_arch,vertices,triangulos);

   // COMPLETAR: práctica 4: invocar  a 'calcularNormales' para el cálculo de normales
   // .................

}

// ****************************************************************************
// Clase 'Cubo

Cubo::Cubo()
:  MallaInd( "cubo 8 vértices" )
{

   vertices =
      {  { -1.0, -1.0, -1.0 }, // 0
         { -1.0, -1.0, +1.0 }, // 1
         { -1.0, +1.0, -1.0 }, // 2
         { -1.0, +1.0, +1.0 }, // 3
         { +1.0, -1.0, -1.0 }, // 4
         { +1.0, -1.0, +1.0 }, // 5
         { +1.0, +1.0, -1.0 }, // 6
         { +1.0, +1.0, +1.0 }, // 7
      } ;



   triangulos =
      {  {0,1,3}, {0,3,2}, // X-
         {4,7,5}, {4,6,7}, // X+ (+4)

         {0,5,1}, {0,4,5}, // Y-
         {2,3,7}, {2,7,6}, // Y+ (+2)

         {0,6,4}, {0,2,6}, // Z-
         {1,5,7}, {1,7,3}  // Z+ (+1)
      } ;

}

Tetraedro::Tetraedro()
: MallaInd("tetraedro de 4 vértices")
{
   //asignamos un color al objeto
   Tetraedro::ponerColor({0.5,0.0,1.0});

   vertices =
       {
           {1.0,-1.0,-1.0}, //0
           {1.0,1.0,-1.0}, //1
           {-1.0,-1.0,-1.0}, //2
           {1.0,0.0,1.0} //3
       };

   triangulos =
       {
           {0,1,2},
           {0,1,3},
           {0,2,3},
           {1,2,3},

       };

}

CuboColores::CuboColores()
: MallaInd("cubo de colores")
{
   vertices =
      {  { -1.0, -1.0, -1.0 }, // 0
         { -1.0, -1.0, +1.0 }, // 1
         { -1.0, +1.0, -1.0 }, // 2
         { -1.0, +1.0, +1.0 }, // 3
         { +1.0, -1.0, -1.0 }, // 4
         { +1.0, -1.0, +1.0 }, // 5
         { +1.0, +1.0, -1.0 }, // 6
         { +1.0, +1.0, +1.0 }, // 7
      } ;



   triangulos =
      {  {0,1,3}, {0,3,2}, // X-
         {4,7,5}, {4,6,7}, // X+ (+4)

         {0,5,1}, {0,4,5}, // Y-
         {2,3,7}, {2,7,6}, // Y+ (+2)

         {0,6,4}, {0,2,6}, // Z-
         {1,5,7}, {1,7,3}  // Z+ (+1)
      } ;

      //asignamos un color en formato RGB a cada vertice
      glm::vec3 auxiliar;
      float r,g,b;
      for(int i=0;i<vertices.size();i++){
         auxiliar=vertices.at(i);
         
         if(auxiliar[0]==1.0){
            //X+
            r=1.0;
         }
         else{
            //X-
            r=0.0;
         }

         if(auxiliar[1]==1.0){
            //Y+
            g=1.0;
         }
         else{
            //Y-
            g=0.0;
         }

         if(auxiliar[2]==1.0){
            //Z+
            b=1.0;
         }
         else{
            //Z-
            b=0.0;
         }
         
         //metemos el color del vertice 
         col_ver.push_back({r,g,b});
      }

}

EstrellaZ::EstrellaZ(unsigned int n) : MallaInd("Estrella Z")
{
   assert(n>1);
   //posiciones de los vertices (dentro del vector 'vertices')

   //metemos primero el centro de la estrella
   vertices.push_back({0.5,0.5,0.0});

   const double pi = 3.14159265358979323846;
   float num_secciones=2.0*pi/float(n);
   float rotacion=pi/float(n);
   for(int i=0;i<n;i++){
      float seccion_actual=i*num_secciones;
      //vertice donde se situa cada una de las n puntas
      vertices.push_back({0.5+0.5*cos(seccion_actual),0.5+0.5*sin(seccion_actual),0.0});
      //vertice situado entre punta y punta
      vertices.push_back({0.5+ 0.25*cos(seccion_actual+rotacion), 0.5 +0.25*sin(seccion_actual+rotacion), 0.0});
   }

   //indices de los vertices de cada uno de los triangulos
   for(int i=1;i<2*n;i++){
      triangulos.push_back({0,i,i+1});
   }
   //ultimo triangulo
   triangulos.push_back({0,1,2*n});

   //colores de los vertices

   //primer vertice: vertice central -> color blanco
   col_ver.push_back({1.0,1.0,1.0});
   //color del resto de vertices == coordenadas en eje X,Y,Z
   float r,g,b;
   glm::vec3 auxiliar;
   for(int i=1;i<vertices.size();i++){
      auxiliar=vertices.at(i);
      r=auxiliar[0];
      g=auxiliar[1];
      b=auxiliar[2];
      col_ver.push_back({r,g,b});
   }



}

CasaX::CasaX() : MallaInd("casa con tejado")
{

  vertices =
      {  {0.0,0.0,0.0},//0
         {0.0,0.0,1.0},//1
         {1.0,0.0,0.0},//2
         {1.0,0.0,1.0},//3
         {0.0,0.75,0.0},//4
         {0.0,0.75,1.0},//5
         {1.0,0.75,0.0},//6
         {1.0,0.75,1.0},//7

         //puntos nuevos necesarios para formar el tejado
         {0.0,1.0,0.5}, //8 (entre 4 y 5)
         {1.0,1.0,0.5} // 9 (entre 6 y 7)
      } ;



   triangulos =
      {  {0,1,5},{0,5,4}, //X-
         {2,3,7},{2,7,6}, //X+
         {1,3,7},{1,7,5}, //Z+
         {0,2,6},{0,6,4}, //Z+


         //triangulos nuevos necesarios para formar el tejado
         {6,7,9},{4,5,8},
         {5,7,9},{5,8,9},
         {6,9,4},{4,8,9}
      } ;

   
   //color de los vertices
   float r,g,b;
   glm::vec3 auxiliar;
   for(int i=0;i<vertices.size();i++){
      auxiliar=vertices.at(i);
      r=auxiliar[0];
      g=auxiliar[1];
      b=auxiliar[2];
      col_ver.push_back({r,g,b});
   }
   
   
   
}

MallaTriangulo::MallaTriangulo() : MallaInd("malla triangulo")
{
   vertices=
   {
      {-0.5,0.0,0.0},
      {0.5,0.0,0.0},
      {0.0,1.414,0.0}
   };

   triangulos.push_back({0,1,2});
}

MallaCuadrado::MallaCuadrado() : MallaInd("cuadrado centrado en el origen y perpendicular a Z")
{
   vertices=
   {
      {1.0,1.0,0.0},//0
      {-1.0,1.0,0.0},//1
      {-1.0,-1.0,0.0},//2
      {1.0,-1.0,0.0}//3
   };

   triangulos={
      {0,1,2},{0,2,3}
   };
}

MallaPiramideL::MallaPiramideL() : MallaInd("malla con forma de piramide y base con forma de L")
{
   vertices={
      //primero formamos la base (forma de L)
      {0.0,0.0,0.0},//0
      {2.0,0.0,0.0},//1
      {2.0,0.0,2.0},//2
      {1.0,0.0,2.0},//3
      {1.0,0.0,1.0},//4
      {0.0,0.0,1.0},//5

      //vertice extra al que llegan todos los triangulos
      {1.0,2.0,1.0},//6 (incrementar en 2 la altura del vertice 4)
   };

   triangulos={
      //triangulos que conforman la base
      {0,5,4},
      {4,3,2},
      {1,0,2},

      //triangulos que forman las caras laterales
      {2,3,6},{1,2,6},{1,0,6},{0,5,6},{5,4,6},{3,4,6}
   };

   //asignamos un color a cada vertice (=coordenadas x,y,z)
   float r,g,b;
   glm::vec3 auxiliar;
   for(int i=0;i<vertices.size();i++){
      auxiliar=vertices.at(i);
      r=auxiliar[0];
      g=auxiliar[1];
      b=auxiliar[2];
      col_ver.push_back({r,g,b});
   }
}

PiramideEstrellaZ::PiramideEstrellaZ(unsigned int n) : MallaInd("piramide estrella Z")
{
   assert(n>1);
   //vertice nuevo...
   vertices.push_back({0.5,0.5,0.5});

   //formamos la base de la figura (estrella Z)
   //posiciones de los vertices (dentro del vector 'vertices')

   //metemos primero el centro de la estrella
   vertices.push_back({0.5,0.5,0.0});

   const double pi = 3.14159265358979323846;
   float num_secciones=2.0*pi/float(n);
   float rotacion=pi/float(n);
   for(int i=0;i<n;i++){
      float seccion_actual=i*num_secciones;
      //vertice donde se situa cada una de las n puntas
      vertices.push_back({0.5+0.5*cos(seccion_actual),0.5+0.5*sin(seccion_actual),0.0});
      //vertice situado entre punta y punta
      vertices.push_back({0.5+ 0.25*cos(seccion_actual+rotacion), 0.5 +0.25*sin(seccion_actual+rotacion), 0.0});
   }

   //indices de los vertices de cada uno de los triangulos
   for(int i=2;i<2*n+1;i++){
      triangulos.push_back({0,i,i+1});
      triangulos.push_back({1,i,i+1});
   }
   //ultimos triangulos
   triangulos.push_back({0,2,2*n+1});
   triangulos.push_back({1,2,2*n+1});

   
   //colores de los vertices
   col_ver.push_back({1.0,1.0,1.0});
   //primer vertice: vertice central -> color blanco
   col_ver.push_back({1.0,1.0,1.0});
   //color del resto de vertices == coordenadas en eje X,Y,Z
   float r,g,b;
   glm::vec3 auxiliar;
   for(int i=2;i<vertices.size();i++){
      auxiliar=vertices.at(i);
      r=auxiliar[0];
      g=auxiliar[1];
      b=auxiliar[2];
      col_ver.push_back({r,g,b});
   }
   
}

RejillaY::RejillaY(unsigned int m,unsigned int n) : MallaInd("Rejilla Y")
{
   assert(m>1 && n>1); // m --> nº de celdas en eje X // n --> nº de celdas en eje Z

   //vertices de la rejilla
   for(float i=0;i<m;i++){
      for(float j=0;j<n;j++){
         vertices.push_back({float(i/(m-1)),0.0,float(j/(n-1))});
      }
   }

   //indices de los vertices
   for(int i=0;i<m-1;i++){
      for(int j=0;j<n-1;j++){
         int index=i*n+j;
         triangulos.push_back({index,index+1,index+n});
         triangulos.push_back({index+1,index+n+1,index+n});
      }
   }

   //color de cada vertice
   float r,g,b;
   glm::vec3 auxiliar;
   for(int i=0;i<vertices.size();i++){
      auxiliar=vertices.at(i);
      r=auxiliar[0];
      g=auxiliar[1];
      b=auxiliar[2];
      col_ver.push_back({r,g,b});
   }
}

MallaTorre::MallaTorre(unsigned int n) : MallaInd("malla torre")
{
   for(int i=0;i<=n;i++){
      vertices.push_back({0.5,float(i),0.5});
      vertices.push_back({0.5,float(i),-0.5});
      vertices.push_back({-0.5,float(i),-0.5});
      vertices.push_back({-0.5,float(i),0.5});
   }

   //indices de los vertices que forman cada una de las caras (en sentido antihorario)
   for(int i=0;i<n;i++){
      for(int j=0;j<3;j++){
         int index=4*i+j;
         triangulos.push_back({index,index+1,index+5});
         triangulos.push_back({index,index+5,index+4});
      }
      //tomando el ultimo vertice de la base de cada cubo...
      triangulos.push_back({4*i+3,4*i,4*i+4});
      triangulos.push_back({4*i+3,4*i+4,4*i+7});
   }

   //color de cada vertice
   float r,g,b;
   glm::vec3 auxiliar;
   for(int i=0;i<vertices.size();i++){
      auxiliar=vertices.at(i);
      r=auxiliar[0];
      g=auxiliar[1];
      b=auxiliar[2];
      col_ver.push_back({r,g,b});
   }
}; 
// -----------------------------------------------------------------------------------------------

