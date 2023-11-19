// *********************************************************************
// **
// ** Asignatura: INFORMÁTICA GRÁFICA
// ** 
// ** Mallas indexadas (implementación)
// ** Copyright (C) 2016-2023 Carlos Ureña
// **
// ** Implementación de las clases 
// **    + MallaRevol: malla indexada de triángulos obtenida por 
// **      revolución de un perfil (derivada de MallaInd)
// **    + MallaRevolPLY: malla indexada de triángulos, obtenida 
// **      por revolución de un perfil leído de un PLY (derivada de MallaRevol)
// **    + algunas clases derivadas de MallaRevol
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
#include "lector-ply.h"
#include "malla-revol.h"

using namespace std ;

// *****************************************************************************




// Método que crea las tablas de vértices, triángulos, normales y cc.de.tt.
// a partir de un perfil y el número de copias que queremos de dicho perfil.
void MallaRevol::inicializar
(
   const std::vector<glm::vec3> & perfil,     // tabla de vértices del perfil original
   const unsigned               num_copias  // número de copias del perfil
)
{
   using namespace glm ;
   
   // COMPLETAR: práctica 2: implementar algoritmo de creación de malla de revolución
   //
   // Escribir el algoritmo de creación de una malla indexada por revolución de un 
   // perfil, según se describe en el guion de prácticas.
   //
   // ............................... 

   //Creamos la tabla de vertices de las instancias del perfil (0 a n-1)
   int m=perfil.size();
   for(int i=0;i<num_copias;i++){
      //estamos en la instancia del perfil 'i'...
      float rotacion=float(2*M_PI*float(i)/float(num_copias-1));
      for(int j=0;j<m;j++){
         //vertice de indice 'j' de la instancia de perfil 'i'...
         //tomamos el vertice de indice 'j' del perfil original...
         glm::vec3 vertice_original=perfil[j];
         vertices.push_back({vertice_original[0]*cos(rotacion)+vertice_original[2]*sin(rotacion),vertice_original[1],-vertice_original[0]*sin(rotacion)+vertice_original[2]*cos(rotacion)});
      }
   }

   //Creamos la tabla de caras
   for(int i=0;i<num_copias-1;i++){
      //estamos en la instancia del perfil 'i'...
      float rotacion=float(2*M_PI*float(i)/float(num_copias-1));
      for(int j=0;j<m-1;j++){
         //vertice de indice 'j' de la instancia de perfil 'i'...
         int k=i*m+j;
         triangulos.push_back({k,k+m,k+m+1});
         triangulos.push_back({k,k+m+1,k+1});
      }
   }





}

// -----------------------------------------------------------------------------
// constructor, a partir de un archivo PLY

MallaRevolPLY::MallaRevolPLY
(
   const std::string & nombre_arch,
   const unsigned      nperfiles
)
{
   ponerNombre( std::string("malla por revolución del perfil en '"+ nombre_arch + "'" ));
   // COMPLETAR: práctica 2: crear la malla de revolución
   // Leer los vértice del perfil desde un PLY, después llamar a 'inicializar'
   // ...........................
   vector<glm::vec3> perfil;
   LeerVerticesPLY(nombre_arch,perfil);
   inicializar(perfil,nperfiles);

}

Cilindro::Cilindro(const int num_verts_per, const unsigned nperfiles){
   //primero creamos el perfil de partida
   vector<glm::vec3> perfil;

   perfil.push_back({0.0,0.0,0.0});

   
   for(int i=0;i<(num_verts_per-2);i++){
      //recorremos la altura del perfil del cilindro
      perfil.push_back({0.5,float(i/(num_verts_per-3)),0.0});
   }

   perfil.push_back({0.0,1.0,0.0});

   //ya tenemos el perfil, creamos las tablas de vertices y triangulos
   inicializar(perfil,nperfiles);

}

Cono::Cono(const int num_verts_per, const unsigned nperfiles){
   //primero creamos el perfil de partida
   vector<glm::vec3> perfil;

   perfil.push_back({0.0,0.0,0.0});

   //calculo el tamaña de cada paso a dar
   float paso=1.0f/(num_verts_per-2);
   //defino el punto de partida y el vector que utilizo en la traslacion que realizaré
   glm::vec3 punto_partida={1.0,0.0,0.0};
   glm::vec3 vector_desplazamiento={-1.0,1.0,0.0};

   for(int i=0;i<(num_verts_per-1);i++){
      perfil.push_back(punto_partida+i*paso*vector_desplazamiento);
   }

   //ya tenemos el perfil, creamos las tablas de vertices y triangulos
   inicializar(perfil,nperfiles);

}

Esfera::Esfera(const int num_verts_per, const unsigned nperfiles){
   //primero creamos el perfil de partida
   vector<glm::vec3> perfil;

   //vertice_partida={0.0,-1.0,0.0};
   for(int i=0;i<num_verts_per;i++){
      //angulo de rotacion
      float alpha=i*M_PI/float(num_verts_per-1);
      perfil.push_back({sinf(alpha),-cosf(alpha),0.0});
   }

   //ya tenemos el perfil, creamos las tablas de vertices y triangulos
   inicializar(perfil,nperfiles);

}




