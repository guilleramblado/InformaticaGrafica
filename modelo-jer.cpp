#include "modelo-jer.h"
#include "malla-revol.h"
#include "objeto3d.h"
using namespace glm; //incluye las transformaciones geometricas que usaremos

Pierna::Pierna(){
    //construyo el pie
    NodoGrafoEscena *pie = new NodoGrafoEscena();
    pie->agregar(scale(vec3{0.4,0.2,0.7}));
    pie->agregar(new Esfera(20,20));
    //color del pie
    pie->ponerColor({0.0,0.0,0.0});

    //construyo finalmente la pierna
    agregar(pie);
    agregar(scale(vec3{0.7,1.5,0.5}));
    agregar(translate(vec3{0.0,0.0,-0.5}));
    agregar(new Cilindro(4,40));
}

Brazo::Brazo(){
    //construimos la mano
    NodoGrafoEscena *mano = new NodoGrafoEscena();
    mano->agregar(scale(vec3{0.5,0.5,0.5}));
    mano->agregar(new Esfera(20,20));
    mano->ponerColor({0.0,0.0,0.0});

    //construimos el hombro
    NodoGrafoEscena *hombro = new NodoGrafoEscena();
    hombro->agregar(translate(vec3{0.0,1.5,0.0}));
    hombro->agregar(scale(vec3{0.7,0.5,0.7}));
    hombro->agregar(new Esfera(20,20));

    //construimos finalmente el brazo
    NodoGrafoEscena *brazo_sin_mano = new NodoGrafoEscena();
    brazo_sin_mano->agregar(scale(vec3{1.0,1.5,1.0}));
    brazo_sin_mano->agregar(new Cilindro(4,40));
    brazo_sin_mano->ponerColor({0.5,0.5,0.5});


    agregar(scale(vec3{1.0,1.0,0.6}));
    agregar(mano);
    agregar(translate(vec3{0.0,0.5,0.0}));
    agregar(brazo_sin_mano);
    agregar(hombro);
}

Pecho:: Pecho(){
    agregar(scale(vec3{1.1,1.5,0.6})); //reduzco anchura pecho
    agregar(new Cubo());
    ponerColor({0.0,0.0,1.0});
}

Ojo::Ojo(){
    agregar(scale(vec3{0.4,0.5,0.5}));
    agregar(new Esfera(20,20));
}

Cabeza::Cabeza(){
    NodoGrafoEscena *ojos = new NodoGrafoEscena();
    ojos->agregar(new Ojo());
    ojos->agregar(translate(vec3{1,0.0,0.0}));
    ojos->agregar(new Ojo());
    ojos->ponerColor({0.3,0.2,0.1});

    agregar(new Cubo());
    agregar(translate(vec3{-0.5,0.0,1.0}));
    agregar(ojos);
}



Robot::Robot(const float angulo_rotacion, const float longitud_cuello, const float rotacion_cabeza){
    //la longitud del cuello sera un valor entre 0.5 y 1
    //la rotacion de los brazos y piernas tomará valores en [-45,45]
    //la rotacion de la cabeza podrá tomar cualquier valor
    assert(longitud_cuello>=0.5 && longitud_cuello<=1);
    assert(angulo_rotacion>=-45 && angulo_rotacion<=45);

    //-----------PIERNAS DEL ROBOT--------------

    //Pierna izquierda
    NodoGrafoEscena *pierna_izquierda_rotada = new NodoGrafoEscena();

    unsigned i = pierna_izquierda_rotada->agregar(translate(vec3{0.0,1.5,-0.25})* rotate(radians(-angulo_rotacion),vec3{1,0,0}) * translate(vec3{0.0,-1.5,0.25})); 
    pierna_izquierda_rotada->agregar(new Pierna());

    //Pierna derecha
    NodoGrafoEscena *pierna_derecha_rotada = new NodoGrafoEscena();
    unsigned j = pierna_derecha_rotada->agregar(translate(vec3{1.5,1.5,-0.25})* rotate(radians(angulo_rotacion),vec3{1,0,0}) * translate(vec3{-1.5,-1.5,0.25})); 
    pierna_derecha_rotada->agregar(translate(vec3{1.5,0.0,0.0}));
    pierna_derecha_rotada->agregar(new Pierna());

    NodoGrafoEscena *piernas= new NodoGrafoEscena();
    piernas->agregar(pierna_izquierda_rotada);
    piernas->agregar(pierna_derecha_rotada);


    //-------------TORSO DEL ROBOT------------

    //Brazos
    NodoGrafoEscena *brazo_izq_rotado = new NodoGrafoEscena();
    unsigned k = brazo_izq_rotado->agregar(translate(vec3{0.0,2.0,0.0})* rotate(radians(angulo_rotacion),vec3{1,0,0}) * translate(vec3{0.0,-2.0,0.0}));
    brazo_izq_rotado->agregar(new Brazo());

    NodoGrafoEscena *brazo_der_rotado = new NodoGrafoEscena();
    unsigned l = brazo_der_rotado->agregar(translate(vec3{3.6,2.0,0.0})* rotate(radians(-angulo_rotacion),vec3{1,0,0}) * translate(vec3{-3.6,-2.0,0.0}));
    brazo_der_rotado->agregar(translate(vec3{3.6,0.0,0.0}));
    brazo_der_rotado->agregar(new Brazo());

    //Cuello
    NodoGrafoEscena *cuello= new NodoGrafoEscena();
    unsigned m = cuello->agregar(scale(vec3{0.7,longitud_cuello,0.7}));
    cuello->agregar(new Cilindro(4,20));

    //Cabeza+cuello
    NodoGrafoEscena *cabezacuello = new NodoGrafoEscena();
    cabezacuello->agregar(cuello);
    //cabezacuello->agregar(translate(vec3{0,1.5+0.7+longitud_cuello,0})* rotate(radians(rotacion_cabeza),vec3{0,1,0}) * translate(vec3{0,-1.5-0.7-longitud_cuello,0}));
    unsigned n = cabezacuello->agregar(translate(vec3{0,0.7+longitud_cuello,0})); //movemos la cabeza para ajustarla al cuello
    cabezacuello->agregar(scale(vec3{0.8,0.8,0.8}));
    unsigned o = cabezacuello->agregar(rotate(radians(rotacion_cabeza),vec3{0,1,0}));
    cabezacuello->agregar(new Cabeza());

    //torso
    NodoGrafoEscena *torso = new NodoGrafoEscena();
    torso->agregar(brazo_izq_rotado);
    torso->agregar(brazo_der_rotado);
    torso->agregar(translate(vec3{1.8,0.5,0.0})); //ajustamos cabeza+cuello+pecho a los BRAZOS
    torso->agregar(new Pecho());
    torso->agregar(translate(vec3{0,1.5,0})); //ajustamos cabeza+cuello al PECHO
    torso->agregar(cabezacuello);

    //pegamos el torso con las piernas
    agregar(torso);
    agregar(translate(vec3{1.05,-2.3,0.25}));
    agregar(piernas);

    //apuntamos a las matrices de transformacion afectadas por algun grado de libertad

    //matrices afectadas por la rotacion de brazos y piernas
    pm_rotacion_pierna_izquierda = pierna_izquierda_rotada->leerPtrMatriz(i);
    pm_rotacion_pierna_derecha = pierna_derecha_rotada->leerPtrMatriz(j);
    pm_rotacion_brazo_izquierdo = brazo_izq_rotado->leerPtrMatriz(k);
    pm_rotacion_brazo_derecho = brazo_der_rotado->leerPtrMatriz(l);
    //matrices afectadas por la longitud del cuello
    pm_longitud_cuello = cuello->leerPtrMatriz(m);
    pm_traslacion_cabeza = cabezacuello->leerPtrMatriz(n);
    pm_rotacion_cabeza = cabezacuello->leerPtrMatriz(o);
    
}

unsigned Robot::leerNumParametros() const {
    //nº de grados de libertad empleados
    return 3;
}

void Robot::actualizarEstadoParametro( const unsigned iParam, const float t_sec ){
    unsigned num_parametros = leerNumParametros();
    //los indices de los parametros iran desde 0 hasta n-1
    assert(iParam<num_parametros);
    switch(iParam){
        case 0:
        {
            /*
            - Grado de libertad: rotacion de brazos y piernas
            - Toma valores en [-45,45]
            - Su valor cambiará, en función del valor de tiempo asociado, de forma oscilante,
            oscilando entre los valores -45 y 45
            */
           float min=-45.0,max=45.0;
           float a = (min+max)/2;
           float b = (max-min)/2;
           fijarRotacionPiernasBrazos(a+b*sin(2*M_PI*2*t_sec));
           break;
        }
        case 1:
        {
            /*
            - Grado de libertad: longitud cuello
            - Toma valores en [0.5,1]
            - Cambia de forma oscilante
            */
           float min=0.5,max=1.0;
           float a = (min+max)/2;
           float b = (max-min)/2;
           fijarLongitudCuello(a+b*sin(2*M_PI*2*t_sec));
           break;
        }
        case 2:
        {
            /*
            - Grado de libertad: rotacion de cabeza
            - Toma cualquier valor
            - Su valor cambiará de forma lineal (podemos rotar la cabeza tanto como queramos)
            En este caso, partimos de 0 grados, incrementando el grado de rotacion de 15 en 15
            */
           fijarRotacionCabeza(15.0*t_sec);
           break;
        }

    }
}

void Robot::fijarRotacionPiernasBrazos(const float angulo_nuevo){
    //modificar la matriz de rotacion dependiente de dicho grado de libertad 

    //rotacion piernas
    *pm_rotacion_pierna_izquierda = translate(vec3{0.0,1.5,-0.25})* rotate(radians(-angulo_nuevo),vec3{1,0,0}) * translate(vec3{0.0,-1.5,0.25});
    *pm_rotacion_pierna_derecha = translate(vec3{1.5,1.5,-0.25})* rotate(radians(angulo_nuevo),vec3{1,0,0}) * translate(vec3{-1.5,-1.5,0.25});
    //rotacion brazos
    *pm_rotacion_brazo_izquierdo = translate(vec3{0.0,2.0,0.0})* rotate(radians(angulo_nuevo),vec3{1,0,0}) * translate(vec3{0.0,-2.0,0.0});
    *pm_rotacion_brazo_derecho = translate(vec3{3.6,2.0,0.0})* rotate(radians(-angulo_nuevo),vec3{1,0,0}) * translate(vec3{-3.6,-2.0,0.0});
}

void Robot::fijarLongitudCuello(const float longitud_nueva){
    //si utilizamos un nuevo valor para dicho parametro, habra que modificar las dos matrices de transformacion afectadas

    //matriz que permite definir la longitud del cuello
    *pm_longitud_cuello = scale(vec3{0.7,longitud_nueva,0.7});
    //matriz de traslacion que permite ajustar la cabeza al cuello del robot
    *pm_traslacion_cabeza = translate(vec3{0,0.7+longitud_nueva,0});
}

void Robot::fijarRotacionCabeza(const float angulo_cabeza_nuevo){
    *pm_rotacion_cabeza = rotate(radians(angulo_cabeza_nuevo),vec3{0,1,0});
}