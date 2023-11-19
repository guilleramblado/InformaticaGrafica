#include "grafo-escena.h"
#include <cmath>

class Pierna: public NodoGrafoEscena {
    public:
        Pierna();
};

class Brazo: public NodoGrafoEscena {
    public: 
        Brazo();
};

class Pecho: public NodoGrafoEscena {
    public:
        Pecho();
};

class Ojo: public NodoGrafoEscena {
    public:
        Ojo();
};

class Cabeza: public NodoGrafoEscena {
    public:
        Cabeza();
};


class Robot: public NodoGrafoEscena {
    //punteros a matrices afectadas por los grados de libertad
    protected:
        //mtrices de rotacion de las piernas
        glm::mat4 *pm_rotacion_pierna_izquierda = nullptr; 
        glm::mat4 *pm_rotacion_pierna_derecha = nullptr; 
        //matrices de rotacion de los brazos
        glm::mat4 *pm_rotacion_brazo_izquierdo = nullptr; 
        glm::mat4 *pm_rotacion_brazo_derecho = nullptr; 
        //matrices de escalado del cuello y traslacion de cabeza (para ajustarla debidamente al cuello)
        glm::mat4 *pm_longitud_cuello = nullptr;
        glm::mat4 *pm_traslacion_cabeza = nullptr;
        //matriz de rotacion de la cabeza
        glm::mat4 *pm_rotacion_cabeza = nullptr;

        public:
            Robot(const float angulo_rotacion, const float longitud_cuello, const float rotacion_cabeza);
            virtual unsigned leerNumParametros() const ;
            virtual void actualizarEstadoParametro( const unsigned iParam, const float t_sec );
            void fijarRotacionPiernasBrazos (const float angulo_nuevo);
            void fijarLongitudCuello (const float longitud_nueva);
            void fijarRotacionCabeza (const float angulo_cabeza_nuevo);
};
