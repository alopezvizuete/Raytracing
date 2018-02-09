#ifdef _OS_X_
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>	

#elif defined(WIN32)
#include <windows.h>
#include "GL/gl.h"
#include "GL/glu.h"
#include "GL/glut.h"

#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif


#include "Scene.h"
#include "RayTrace.h"
#include <iostream>

//Matriz de Traslacion
Matrix matrizTraslacion(double X, double Y, double Z) {
	Matrix traslacion;
	traslacion._14 = X;
	traslacion._24 = Y;
	traslacion._34 = Z;
	return traslacion;
}
//Matriz de Rotacion en el eje X
Matrix matrizRotacionX(double X)
{
	X = (X * 2 * (3.1415)) / 360.0;
	Matrix rotacionX;
	rotacionX._22 = cos(X);
	rotacionX._23 = -sin(X);
	rotacionX._32 = sin(X);
	rotacionX._33 = cos(X);
	return rotacionX;
}
//Matriz de Rotacion en el eje Y
Matrix matrizRotacionY(double Y)
{
	Y = (Y * 2 * (3.1415)) / 360.0;
	Matrix rotacionY;
	rotacionY._11 = cos(Y);
	rotacionY._13 = sin(Y);
	rotacionY._31 = -sin(Y);
	rotacionY._33 = cos(Y);
	return rotacionY;
}
//Matriz de Rotacion en el eje Z
Matrix matrizRotacionZ(double Z)
{
	Z = (Z * 2 * (3.1415)) / 360.0;
	Matrix rotacionZ;
	rotacionZ._11 = cos(Z);
	rotacionZ._12 = -sin(Z);
	rotacionZ._21 = sin(Z);
	rotacionZ._22 = cos(Z);
	return rotacionZ;
}
//Matriz Scale
Matrix escala(double X, double Y, double Z) {
	Matrix escala;
	escala._11 = X;
	escala._22 = Y;
	escala._33 = Z;
	return escala;

}

//Funcion que calcula el modulo de un vector
float modulo(Vector a) {
	float b;
	b = sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
	return b;
}



//Iluminacion Phong
Vector iluminacionPhong(Vector posPixel, Vector N, Vector pos, Vector posLuz, Vector I, Vector Ia, Vector Kd, Vector Ks, float shine) {

	Vector V = (pos - posPixel).Normalize(); //Vector de Visibilidad
	Vector L = (posLuz - posPixel).Normalize(); //Vector de Luz
	Vector R = (N*(N.Dot(L)) * 2) - L; //Vector reflejado

	return Ia*0.3 + I*Kd*N.Dot(L) + I * Ks * pow(R.Dot(V), shine); //Devolvemos el color de sombreado


}
//Calculamos interseccion con la esfera como se explico en clase.
bool intersectionSphere(Vector pos, Vector dir, float radio, Vector centro, double &t) {
	Vector vec = centro - pos; //Calculamos la vector hacia la esfera
	double tDist = vec.Dot(dir); //Miramos si su distancia es positiva, de ser asi seguimos
	if (tDist < 0)
		return false;
	double d2 = vec.Dot(vec) - tDist*tDist; //obtenemos la distancia que existe
	double dist = sqrt(radio*radio - d2);  //Entre el punto que interseccionamos y el centro
	double zDistance = tDist - dist;  //Restamos la distancia obtenida a la distancia al centro
	if (d2 < radio*radio) { //Si la distancia de interseccion esta dentro del radio, obtenemos interseccion
		t = zDistance; //Devolvemos la longitud para crear el rayo
		return true;
	}
	return false;
}

//Interseccion con triangulo segun el algoritmo Moller-Trumbore
// https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
bool intersectionTriangulo(Vector pos, Vector dir, Vector v1, Vector v2, Vector v3, float &out, Vector &CoorBar) {

	Vector e1, e2;  //Edge1, Edge2
	Vector P, Q, T;
	float det, inv_det;
	double u, v;
	double t;
	float EPSILON = 0.000001;

	e1 = v2 - v1;
	e2 = v3 - v1;
	P = dir.Cross(e2);
	det = e1.Dot(P);

	//NOT CULLING
	if (det > -EPSILON && det < EPSILON) return false;
	inv_det = 1.0 / det;

	//calculate distance from V1 to ray origin
	T = pos - v1;

	//Calculate u parameter and test bound
	u = T.Dot(P) * inv_det;
	//The intersection lies outside of the triangle
	if (u < 0.0 || u > 1.0) return false;

	//Prepare to test v parameter
	Q = T.Cross(e1);

	//Calculate V parameter and test bound
	v = dir.Dot(Q)* inv_det;
	//The intersection lies outside of the triangle
	if (v < 0.0 || u + v  > 1.0) return false;

	t = e2.Dot(Q)* inv_det;

	if (t > EPSILON) { //ray intersection
		CoorBar = Vector(1 - (u + v), u, v);
		out = t;
		return true;
	}
	// No hit, no win
	return false;

}

//Funcion para Shadow Ray
//Realizamos el mismo algoritmo que para RayTracing
//Pero en cuanto obtenemos una interseccion, devolvemos un resultado positivo
//Sino hay interseccion, devolvemos resultado negativo
bool haySombra(Vector pos, Vector dir,float dist, Scene &la_escena) {
	for (unsigned int i = 0; i < la_escena.GetNumObjects(); i++) {
		SceneObject* obj = la_escena.GetObject(i);
		if (obj->IsSphere()) {
			SceneSphere* esfera = static_cast<SceneSphere*> (obj);
			//Matrix scale = escala(esfera->scale.x, esfera->scale.y, esfera->scale.z);
			Matrix rotacionX = matrizRotacionX(esfera->rotation.x);
			Matrix rotacionY = matrizRotacionY(esfera->rotation.y);
			Matrix rotacionZ = matrizRotacionZ(esfera->rotation.z);
			Matrix traslacion = matrizTraslacion(esfera->position.x, esfera->position.y, esfera->position.z);
			Vector newPos = traslacion *  rotacionX * rotacionY * rotacionZ * esfera->center;
			double t = 0;
			if (intersectionSphere(pos, dir, esfera->radius, newPos, t)) 
				if(t<dist)
					return true;
		}
		if (obj->IsTriangle()) {
			SceneTriangle* triangulo = static_cast<SceneTriangle*> (obj);
			Vector v0 = triangulo->vertex[0];
			Vector v1 = triangulo->vertex[1];
			Vector v2 = triangulo->vertex[2];
			//Rotacion
			Matrix rotacionX = matrizRotacionX(triangulo->rotation.x);
			Matrix rotacionY = matrizRotacionY(triangulo->rotation.y);
			Matrix rotacionZ = matrizRotacionZ(triangulo->rotation.z);
			Matrix traslacion = matrizTraslacion(triangulo->position.x, triangulo->position.y, triangulo->position.z);
			Matrix scale = escala(triangulo->scale.x, triangulo->scale.y, triangulo->scale.z);
			v0 = traslacion *  rotacionX * rotacionY * rotacionZ *scale * triangulo->vertex[0];
			v1 = traslacion *  rotacionX * rotacionY * rotacionZ *scale * triangulo->vertex[1];
			v2 = traslacion *  rotacionX * rotacionY * rotacionZ *scale * triangulo->vertex[2];
			float t;
			Vector coorBaric;
			if (intersectionTriangulo(pos, dir, v0, v1, v2, t, coorBaric))
				if (t<dist)
					return true;
		}
		
		if (obj->IsModel()) {
			SceneModel* modelo = static_cast<SceneModel*> (obj);
			bool texCargada = false;
			for (int i = 0; i < modelo->GetNumTriangles(); i++) {
				SceneTriangle* triangulo = modelo->GetTriangle(i);
				Vector v0 = triangulo->vertex[0];
				Vector v1 = triangulo->vertex[1];
				Vector v2 = triangulo->vertex[2];
				//Rotacion
				Matrix rotacionX = matrizRotacionX(modelo->rotation.x);
				Matrix rotacionY = matrizRotacionY(modelo->rotation.y);
				Matrix rotacionZ = matrizRotacionZ(modelo->rotation.z);
				Matrix traslacion = matrizTraslacion(modelo->position.x, modelo->position.y, modelo->position.z);
				Matrix scale = escala(modelo->scale.x, modelo->scale.y, modelo->scale.z);
				v0 = traslacion *  rotacionX * rotacionY * rotacionZ *scale * triangulo->vertex[0];
				v1 = traslacion *  rotacionX * rotacionY * rotacionZ *scale * triangulo->vertex[1];
				v2 = traslacion *  rotacionX * rotacionY * rotacionZ *scale * triangulo->vertex[2];
				float t;
				Vector coorBari;
				if (intersectionTriangulo(pos, dir, v0, v1, v2, t, coorBari)) 
					if (t<dist)
						return true;
			}
		}
	}
	return false;
}

//Funcion para generar un numero aleatorio
float numAleatorio(float n) {
	float num = sin(n*12.9898)*43758.5453;
	return num - (int)num;
}

//Lanzamos un rayo al igual que raytracing para obtener el color
Vector MontecarloColor(Vector pos, Vector dir, Scene &la_escena,int reb) {
	if (reb == 0) {
		return Vector(0.0,0.0,0.0);
	}
	for (unsigned int i = 0; i < la_escena.GetNumObjects(); i++) {
		SceneObject* obj = la_escena.GetObject(i);
		if (obj->IsSphere()) {
			SceneSphere* esfera = static_cast<SceneSphere*> (obj);
			//Matrix scale = escala(esfera->scale.x, esfera->scale.y, esfera->scale.z);
			Matrix rotacionX = matrizRotacionX(esfera->rotation.x);
			Matrix rotacionY = matrizRotacionY(esfera->rotation.y);
			Matrix rotacionZ = matrizRotacionZ(esfera->rotation.z);
			Matrix traslacion = matrizTraslacion(esfera->position.x, esfera->position.y, esfera->position.z);
			Vector newPos = traslacion *  rotacionX * rotacionY * rotacionZ * esfera->center;
			double t = 0;
			if (intersectionSphere(pos, dir, esfera->radius, newPos, t)) {
				Vector rayo = pos + dir*t; //creamos el rayo
				Vector N = (rayo - newPos).Normalize(); //Normal en el punto de interseccion
				Vector L = (pos - rayo).Normalize();
				Vector R = (N*(N.Dot(L)) * 2) - L;//Vector reflejo, para las llamadas recursivas
				rayo = rayo + N*0.000001; //Offset para evitar ruido
				SceneMaterial* mat = la_escena.GetMaterial(esfera->material); //Obtenemos el material de la esfera

				Vector difuso; //Vector para guardar el color difuso en funcion de la textura
				if (mat->texture != "") {
					float u = 0.5 + (atan2(N.x, N.z) / (3.1415));  //Si existe textura, calculamos las coordenadas UV
					float v = 0.5 - (asin(N.y) / (3.1415));        // de la esfera y obtenemos el color de la textura
					difuso = mat->GetTextureColor(u, v) / 255;
				}
				//Si no hay textura, guardamos el color difuso del material
				else {
					difuso = mat->diffuse;
				}


				Vector color = difuso; //Variable para ir guardando el color en el punto de interseccion
							  //PARA CADA LUZ
				for (int l = 0; l < la_escena.GetNumLights(); l++) {
					SceneLight* luzPrueba = la_escena.GetLight(0); //Obtenemos la luz correspondiente
					Vector direcLuz = (luzPrueba->position - rayo); //Cogemos la direccion hacia la luz
					if (haySombra(rayo + N*0.0001, direcLuz.Normalize(), modulo(direcLuz), la_escena)) { //Calculamos el ShadorRay, y si hay sombra
						color = color;  //Sumamos unicamente la luz ambiental
					}
					else { //En caso contrario, calculamos el color por Phong y realizamos las llamadas recursivas
						//color = color + iluminacionPhong(rayo, N, pos, luzPrueba->position, luzPrueba->color, Vector(0.0,0.0,0.0), difuso, mat->specular, mat->shininess);
						color = (color*(Vector(1.0, 1.0, 1.0) - mat->reflective));
						if (mat->reflective.x>0.0)
							color = color + (MontecarloColor(rayo, R, la_escena, reb - 1)*mat->reflective);
					}
				}

				return color;
			}

		}
		if (obj->IsTriangle()) {
			SceneTriangle* triangulo = static_cast<SceneTriangle*> (obj);
			Vector v0 = triangulo->vertex[0];
			Vector v1 = triangulo->vertex[1];
			Vector v2 = triangulo->vertex[2];
			//Rotacion
			Matrix rotacionX = matrizRotacionX(triangulo->rotation.x);
			Matrix rotacionY = matrizRotacionY(triangulo->rotation.y);
			Matrix rotacionZ = matrizRotacionZ(triangulo->rotation.z);
			Matrix traslacion = matrizTraslacion(triangulo->position.x, triangulo->position.y, triangulo->position.z);
			Matrix scale = escala(triangulo->scale.x, triangulo->scale.y, triangulo->scale.z);
			v0 = traslacion *  rotacionX * rotacionY * rotacionZ *scale * triangulo->vertex[0];
			v1 = traslacion *  rotacionX * rotacionY * rotacionZ *scale * triangulo->vertex[1];
			v2 = traslacion *  rotacionX * rotacionY * rotacionZ *scale * triangulo->vertex[2];
			float t;
			Vector coorBaric;
			if (intersectionTriangulo(pos, dir, v0, v1, v2, t, coorBaric)) {
				Vector rayo = pos + dir*t; //Creamos el rayo
				SceneMaterial* mat0 = la_escena.GetMaterial(triangulo->material[0]);
				SceneMaterial* mat1 = la_escena.GetMaterial(triangulo->material[1]);
				SceneMaterial* mat2 = la_escena.GetMaterial(triangulo->material[2]);

				// Interpolamos cada componente de los materiales y triangulos a travez de las coordenadas baricentricas
				Vector colorFinalDifuso = ((mat0->diffuse * coorBaric.x + mat1->diffuse * coorBaric.y + mat2->diffuse * coorBaric.z));
				Vector colorFinalEspecular = ((mat0->specular * coorBaric.x + mat1->specular * coorBaric.y + mat2->specular * coorBaric.z));
				float colorFinalBrillo = ((mat0->shininess * coorBaric.x + mat1->shininess * coorBaric.y + mat2->shininess * coorBaric.z));
				Vector N = ((triangulo->normal[0] * coorBaric.x + triangulo->normal[1] * coorBaric.y + triangulo->normal[2] * coorBaric.z));
				Vector L = (pos - rayo).Normalize();
				Vector R = (N*(N.Dot(L)) * 2) - L;//Vector reflejo, para las llamadas recursivas
				Vector reflexionFinal = ((mat0->reflective * coorBaric.x + mat1->reflective * coorBaric.y + mat2->reflective * coorBaric.z));
				rayo = rayo + N*0.000001; //Offset del rayo
										  //Por cada material, comprobamos si hay textura. En caso afirmativo, interpolamos las coordenadas UV del triangulo
										  //Y cambiamos el color difuso del material por su color de textura
				if (mat0->texture != "") {
					float U = triangulo->u[0] * coorBaric.x + triangulo->u[1] * coorBaric.y + triangulo->u[2] * coorBaric.z;
					float V = triangulo->v[0] * coorBaric.x + triangulo->v[1] * coorBaric.y + triangulo->v[2] * coorBaric.z;
					colorFinalDifuso = colorFinalDifuso - mat0->diffuse * coorBaric.x + (mat0->GetTextureColor(U, V) / 255);
				}
				if (mat1->texture != "") {
					float U = triangulo->u[0] * coorBaric.x + triangulo->u[1] * coorBaric.y + triangulo->u[2] * coorBaric.z;
					float V = triangulo->v[0] * coorBaric.x + triangulo->v[1] * coorBaric.y + triangulo->v[2] * coorBaric.z;
					colorFinalDifuso = colorFinalDifuso - mat1->diffuse * coorBaric.y + (mat1->GetTextureColor(U, V) / 255);
				}
				if (mat2->texture != "") {
					float U = triangulo->u[0] * coorBaric.x + triangulo->u[1] * coorBaric.y + triangulo->u[2] * coorBaric.z;
					float V = triangulo->v[0] * coorBaric.x + triangulo->v[1] * coorBaric.y + triangulo->v[2] * coorBaric.z;
					colorFinalDifuso = colorFinalDifuso - mat2->diffuse * coorBaric.z + (mat2->GetTextureColor(U, V) / 255);
				}
				Vector color = colorFinalDifuso;
				//Por cada luz..
				for (int l = 0; l < la_escena.GetNumLights(); l++) {
					SceneLight* luzPrueba = la_escena.GetLight(l);
					Vector direcLuz = (luzPrueba->position - rayo); //Direccion de la interseccion hacia la luz
					if (haySombra(rayo + N*0.0001, direcLuz.Normalize(), modulo(direcLuz), la_escena)) { //Calculamos el ShadorRay,
						color = color; //Si hay sombra, sumamos la componente ambiental
					}
					else { //Si no hay sombra, sumamos la iluminacion Phong y las llamadas recursivas del Raytracing
						//color = color + iluminacionPhong(rayo, N, pos, luzPrueba->position, luzPrueba->color, Vector(0.0,0.0,0.0), colorFinalDifuso, colorFinalEspecular, colorFinalBrillo);
						color = (color*(Vector(1.0, 1.0, 1.0) - reflexionFinal));
						if (reflexionFinal.x>0.0)
							color = color + (MontecarloColor(rayo, R, la_escena, reb - 1)*reflexionFinal);
					}
				}

				//Interpolamos cada caracteristica del material mediante las coordenadas baricentricas
				return color;
			}
		}

		if (obj->IsModel()) {
			SceneModel* modelo = static_cast<SceneModel*> (obj);
			bool texCargada = false;
			for (int i = 0; i < modelo->GetNumTriangles(); i++) {
				SceneTriangle* triangulo = modelo->GetTriangle(i);
				Vector v0 = triangulo->vertex[0];
				Vector v1 = triangulo->vertex[1];
				Vector v2 = triangulo->vertex[2];
				//Rotacion
				Matrix rotacionX = matrizRotacionX(modelo->rotation.x);
				Matrix rotacionY = matrizRotacionY(modelo->rotation.y);
				Matrix rotacionZ = matrizRotacionZ(modelo->rotation.z);
				Matrix traslacion = matrizTraslacion(modelo->position.x, modelo->position.y, modelo->position.z);
				Matrix scale = escala(modelo->scale.x, modelo->scale.y, modelo->scale.z);
				v0 = traslacion *  rotacionX * rotacionY * rotacionZ *scale * triangulo->vertex[0];
				v1 = traslacion *  rotacionX * rotacionY * rotacionZ *scale * triangulo->vertex[1];
				v2 = traslacion *  rotacionX * rotacionY * rotacionZ *scale * triangulo->vertex[2];
				float t;
				Vector coorBaric;
				if (intersectionTriangulo(pos, dir, v0, v1, v2, t, coorBaric)) {
					Vector rayo = pos + dir*t; //Creamos el rayo
											   //Obtenemos los materiales del triangulo
					SceneMaterial* mat0 = la_escena.GetMaterial(triangulo->material[0]);
					SceneMaterial* mat1 = la_escena.GetMaterial(triangulo->material[1]);
					SceneMaterial* mat2 = la_escena.GetMaterial(triangulo->material[2]);

					//Interpolamos cada caracteristica del material mediante las coordenadas baricentricas
					Vector colorFinalDifuso = ((mat0->diffuse * coorBaric.x + mat1->diffuse * coorBaric.y + mat2->diffuse * coorBaric.z));
					Vector colorFinalEspecular = ((mat0->specular * coorBaric.x + mat1->specular * coorBaric.y + mat2->specular * coorBaric.z));
					float colorFinalBrillo = ((mat0->shininess * coorBaric.x + mat1->shininess * coorBaric.y + mat2->shininess * coorBaric.z));
					Vector N = ((triangulo->normal[0] * coorBaric.x + triangulo->normal[1] * coorBaric.y + triangulo->normal[2] * coorBaric.z));
					Vector L = (pos - rayo).Normalize();
					Vector R = (N*(N.Dot(L)) * 2) - L;//Vector reflejo, para las llamadas recursivas
					Vector reflexionFinal = ((mat0->reflective * coorBaric.x + mat1->reflective * coorBaric.y + mat2->reflective * coorBaric.z));
					rayo = rayo + N*0.000001; //Offset del rayo
					Vector difuso;
					//Comprobamos si hay textura, y en caso afirmativo, calculamos las coordenadas UV
					// Y cogemos el color de la textura
					if (mat0->texture != "") {
						float U = triangulo->u[0] * coorBaric.x + triangulo->u[1] * coorBaric.y + triangulo->u[2] * coorBaric.z;
						float V = triangulo->v[0] * coorBaric.x + triangulo->v[1] * coorBaric.y + triangulo->v[2] * coorBaric.z;
						if (V < 0) {
							V = V*-1;
						}
						if (U > 0 && V > 0)
							difuso = mat0->GetTextureColor(U, V) / 255.0;
					}
					else { //Si no hay textura, cogemos el color difuso del material
						difuso = colorFinalDifuso;
					}
					Vector color = difuso;
					for (int l = 0; l < la_escena.GetNumLights(); l++) {
						SceneLight* luzPrueba = la_escena.GetLight(l);
						Vector direcLuz = (luzPrueba->position - rayo); //Direccion de la interseccion hacia la luz
						if (haySombra(rayo + N*0.0001, direcLuz.Normalize(), modulo(direcLuz), la_escena)) { //Calculamos el ShadorRay,
							color = color; //Si hay sombra, devolvemos el color ambiental
						}
						else { //Si no hay sombra, calculamos iluminacion Phong y realizamos las llamadas recursivas del Raytracing
							//color = color + iluminacionPhong(rayo, N, pos, luzPrueba->position, luzPrueba->color, Vector(0.0,0.0,0.0), difuso, colorFinalEspecular, colorFinalBrillo);
							color = (color*(Vector(1.0, 1.0, 1.0) - reflexionFinal));
							if (reflexionFinal.x>0.0)
								color = color + (MontecarloColor(rayo, R, la_escena, reb - 1)*reflexionFinal);
						}
					}
					return color;
				}
			}
		}
	}
	return Vector(0.0, 0.0, 0.0);
}

// Distribucíon de rayos sacada de:
// https://www.shadertoy.com/view/MdyGDW
Vector MontecarloRay(Vector pos, float n, Scene &la_escena, int reb) {
	//Numeros aleatorios para crear la nueva direccion del rayo
	float a = numAleatorio(n) * 6.283*2.0; 
	float b = numAleatorio(-n) * 6.283*2.0;
	//Creamos una direccion con los numeros obtenidos
	Vector p = Vector(sin(a)*cos(b), -sin(a), cos(a)*cos(b));
	//Si esta en la direccion incorrecta, lo corregimos
	if (p.Dot(pos) < 0.0)
		p = p - (pos* (pos.Dot(p) / pos.Dot(pos))*2.0);

	//Lanzamos el rayo para obtener el color en esa zona
	return MontecarloColor(pos, p, la_escena, reb);
}

//Funcion de calculo de color mediante la tecnica Raytracing por rebotes
Vector Raytracing(Vector pos, Vector dir, Scene &la_escena,int rebotes) {

	SceneBackground fondo = la_escena.GetBackground(); // Obtenemos el fondo de la escena para tener el color de fondo y la luz ambiental

	//Variables para controlar la Z de los objetos
	bool intersecciona = false;
	float zpos = 0.0f;

	Vector LuzAmbiental = fondo.ambientLight;
	//Creamos un Vector respuesta, que sera lo que devolvamos en la funcion
	//Por defecto el color de fondo, por si no hay interseccion con ningun objeto, obtenemos el color de fondo
	Vector respuesta = fondo.color;
	//Cuando la funcion recursiva llega a su fin, devolvemos luz ambiental para ir retornando las llamadas recursivas con este color de base
	if (rebotes == 0) {
		return LuzAmbiental*0.3;
	}

	//Por cada objeto..
	for (unsigned int i = 0; i < la_escena.GetNumObjects(); i++) {
		//Obtenemos el objeto correspondiente
		SceneObject* obj = la_escena.GetObject(i);
		//Si el objeto es una esfera
		if (obj->IsSphere()) { 

			SceneSphere* esfera = static_cast<SceneSphere*> (obj); //Obtenemos la esfera
			//Matrices de translacion y rotacion
			//Matrix scale = escala(esfera->scale.x, esfera->scale.y, esfera->scale.z);
			Matrix rotacionX = matrizRotacionX(esfera->rotation.x);
			Matrix rotacionY = matrizRotacionY(esfera->rotation.y);
			Matrix rotacionZ = matrizRotacionZ(esfera->rotation.z);
			Matrix traslacion = matrizTraslacion(esfera->position.x, esfera->position.y, esfera->position.z);
			//Obtenemos la nueva posicion de la esfera tras aplicar las matrices
			Vector newPos = traslacion *  rotacionX * rotacionY * rotacionZ * esfera->center;
			
			//Creamos la variable de la distancia a la interseccion y llamamos a la funcion para comprobar el choque con la esfera
			double t = 0;
			if (intersectionSphere(pos, dir, esfera->radius, newPos, t)) //EN CASO AFIRMATIVO
			{
				Vector rayo = pos + dir*t; //creamos el rayo

				if (zpos < rayo.z || intersecciona == false) { //Si para este rayo ya ha chocado con alguno objeto, lo descartamos

					Vector N = (rayo - newPos).Normalize(); //Normal en el punto de interseccion
					Vector L = (pos - rayo).Normalize();
					Vector R = (N*(N.Dot(L)) * 2) - L;//Vector reflejo, para las llamadas recursivas
					rayo = rayo + N*0.000001; //Offset para evitar ruido
					SceneMaterial* mat = la_escena.GetMaterial(esfera->material); //Obtenemos el material de la esfera

					Vector difuso; //Vector para guardar el color difuso en funcion de la textura
					if (mat->texture != "") {
							float u = 0.5 + (atan2(N.x, N.z) / (3.1415));  //Si existe textura, calculamos las coordenadas UV
							float v = 0.5 - (asin(N.y) / (3.1415));        // de la esfera y obtenemos el color de la textura
							difuso = mat->GetTextureColor(u, v)/255;
					}
					//Si no hay textura, guardamos el color difuso del material
					else {
						difuso = mat->diffuse;
					}
					if (Scene::montecarlo) {
						LuzAmbiental = Vector(0.0, 0.0, 0.0);
						for (int m = 0; m < 32; m++) {
							LuzAmbiental = LuzAmbiental + (MontecarloRay(rayo, m, la_escena,2) / 32);
						}
					}


					Vector color; //Variable para ir guardando el color en el punto de interseccion
					//PARA CADA LUZ
					for (int l = 0; l < la_escena.GetNumLights(); l++) {
						SceneLight* luzPrueba = la_escena.GetLight(0); //Obtenemos la luz correspondiente
						Vector direcLuz = (luzPrueba->position - rayo); //Cogemos la direccion hacia la luz
						if (haySombra(rayo + N*0.0001, direcLuz.Normalize(),modulo(direcLuz), la_escena)) { //Calculamos el ShadorRay, y si hay sombra
							color = color + LuzAmbiental*0.3;  //Sumamos unicamente la luz ambiental
						}
						else { //En caso contrario, calculamos el color por Phong y realizamos las llamadas recursivas
							color = color + iluminacionPhong(rayo, N, pos, luzPrueba->position, luzPrueba->color, LuzAmbiental, difuso, mat->specular, mat->shininess);
							color = (color*(Vector(1.0, 1.0, 1.0) - mat->reflective));
							if (mat->reflective.x>0.0)
								color = color + (Raytracing(rayo, R, la_escena, rebotes - 1)*mat->reflective);
						}
					}
					//Actualizamos las variables de comprobacion de Z, y devolvemos el color
					intersecciona = true;
					zpos = rayo.z;
					respuesta = color;
				}
			}
		}

		//Si el objeto es un triangulo
		if (obj->IsTriangle()) {
			SceneTriangle* triangulo = static_cast<SceneTriangle*> (obj); //Obtenemos el triangulo
			//Cogemos los vertices
			Vector v0 = triangulo->vertex[0];
			Vector v1 = triangulo->vertex[1];
			Vector v2 = triangulo->vertex[2];

			//Matriz de rotacion,translacion, escala y se aplica a los vertices del triangulo
			Matrix rotacionX = matrizRotacionX(triangulo->rotation.x);
			Matrix rotacionY = matrizRotacionY(triangulo->rotation.y);
			Matrix rotacionZ = matrizRotacionZ(triangulo->rotation.z);
			Matrix traslacion = matrizTraslacion(triangulo->position.x, triangulo->position.y, triangulo->position.z);
			Matrix scale = escala(triangulo->scale.x, triangulo->scale.y, triangulo->scale.z);
			v0 = traslacion *  rotacionX * rotacionY * rotacionZ *scale * triangulo->vertex[0];
			v1 = traslacion *  rotacionX * rotacionY * rotacionZ *scale * triangulo->vertex[1];
			v2 = traslacion *  rotacionX * rotacionY * rotacionZ *scale * triangulo->vertex[2];

			float t;
			Vector coorBaric; //Vector para guardar las coordenadas baricentricas
			//Comprobamos si hay interseccion con el triangulo
			if (intersectionTriangulo(pos, dir, v0, v1, v2, t, coorBaric)) //EN CASO AFIRMATIVO
			{
				Vector rayo = pos + dir*t; //Creamos el rayo
				if (zpos < rayo.z || intersecciona == false) { //Comprobamos la Z del objeto
					//Obtenemos los materiales de la escena
					SceneMaterial* mat0 = la_escena.GetMaterial(triangulo->material[0]);
					SceneMaterial* mat1 = la_escena.GetMaterial(triangulo->material[1]);
					SceneMaterial* mat2 = la_escena.GetMaterial(triangulo->material[2]);
					Vector color;
					// Interpolamos cada componente de los materiales y triangulos a travez de las coordenadas baricentricas
					Vector colorFinalDifuso = ((mat0->diffuse * coorBaric.x + mat1->diffuse * coorBaric.y + mat2->diffuse * coorBaric.z));
					Vector colorFinalEspecular = ((mat0->specular * coorBaric.x + mat1->specular * coorBaric.y + mat2->specular * coorBaric.z));
					float colorFinalBrillo = ((mat0->shininess * coorBaric.x + mat1->shininess * coorBaric.y + mat2->shininess * coorBaric.z));
					Vector N = ((triangulo->normal[0] * coorBaric.x + triangulo->normal[1] * coorBaric.y + triangulo->normal[2] * coorBaric.z));
					Vector L = (pos - rayo).Normalize();
					Vector R = (N*(N.Dot(L)) * 2) - L;//Vector reflejo, para las llamadas recursivas
					Vector reflexionFinal = ((mat0->reflective * coorBaric.x + mat1->reflective * coorBaric.y + mat2->reflective * coorBaric.z));
					rayo = rayo + N*0.000001; //Offset del rayo
					//Por cada material, comprobamos si hay textura. En caso afirmativo, interpolamos las coordenadas UV del triangulo
					//Y cambiamos el color difuso del material por su color de textura
					if (mat0->texture != "") {
							float U = triangulo->u[0] * coorBaric.x + triangulo->u[1] * coorBaric.y + triangulo->u[2] * coorBaric.z;
							float V = triangulo->v[0] * coorBaric.x + triangulo->v[1] * coorBaric.y + triangulo->v[2] * coorBaric.z;
							colorFinalDifuso = colorFinalDifuso- mat0->diffuse * coorBaric.x + (mat0->GetTextureColor(U, V)/255);
					}
					if (mat1->texture != "") {
							float U = triangulo->u[0] * coorBaric.x + triangulo->u[1] * coorBaric.y + triangulo->u[2] * coorBaric.z;
							float V = triangulo->v[0] * coorBaric.x + triangulo->v[1] * coorBaric.y + triangulo->v[2] * coorBaric.z;
							colorFinalDifuso = colorFinalDifuso - mat1->diffuse * coorBaric.y + (mat1->GetTextureColor(U, V) / 255);
					}
					if (mat2->texture != "") {
							float U = triangulo->u[0] * coorBaric.x + triangulo->u[1] * coorBaric.y + triangulo->u[2] * coorBaric.z;
							float V = triangulo->v[0] * coorBaric.x + triangulo->v[1] * coorBaric.y + triangulo->v[2] * coorBaric.z;
							colorFinalDifuso = colorFinalDifuso - mat2->diffuse * coorBaric.z + (mat2->GetTextureColor(U, V) / 255);
					}

					if (Scene::montecarlo) {
						LuzAmbiental = Vector(0.0, 0.0, 0.0);
						for (int m = 0; m < 32; m++) {
							LuzAmbiental = LuzAmbiental + (MontecarloRay(rayo, m,la_escena,2)/32);
						}
					}

					//Por cada luz..
					for (int l = 0; l < la_escena.GetNumLights(); l++) {
						SceneLight* luzPrueba = la_escena.GetLight(l);
						Vector direcLuz = (luzPrueba->position - rayo); //Direccion de la interseccion hacia la luz
						if (haySombra(rayo+N*0.0001, direcLuz.Normalize(), modulo(direcLuz), la_escena)) { //Calculamos el ShadorRay,
							color = color + LuzAmbiental*0.3; //Si hay sombra, sumamos la componente ambiental
						}
						else { //Si no hay sombra, sumamos la iluminacion Phong y las llamadas recursivas del Raytracing
							color = color + iluminacionPhong(rayo, N, pos, luzPrueba->position, luzPrueba->color, LuzAmbiental, colorFinalDifuso, colorFinalEspecular, colorFinalBrillo);
							color = (color*(Vector(1.0, 1.0, 1.0) - reflexionFinal));
							if (reflexionFinal.x>0.0)
								color = color + (Raytracing(rayo, R, la_escena, rebotes - 1)*reflexionFinal);
						}
					}
					//Actualizamos profundidad
					intersecciona = true;
					zpos = rayo.z;
					//Devolvemos el color calculado
					respuesta = color;
				}

			}
		}

		if (obj->IsModel()) {
			SceneModel* modelo = static_cast<SceneModel*> (obj);  //Obtenemos el modelo
			for (int i = 0; i < modelo->GetNumTriangles(); i++) { //POR CADA TRIANGULO DEL OBJETO

				SceneTriangle* triangulo = modelo->GetTriangle(i); //Cogemos el triangulo
				Vector v0 = triangulo->vertex[0];
				Vector v1 = triangulo->vertex[1];
				Vector v2 = triangulo->vertex[2];

				//Rotacion, traslacion, escala y su aplicacion a los vertices
				Matrix rotacionX = matrizRotacionX(modelo->rotation.x);
				Matrix rotacionY = matrizRotacionY(modelo->rotation.y);
				Matrix rotacionZ = matrizRotacionZ(modelo->rotation.z);
				Matrix traslacion = matrizTraslacion(modelo->position.x, modelo->position.y, modelo->position.z);
				Matrix scale = escala(modelo->scale.x, modelo->scale.y, modelo->scale.z);
				v0 = traslacion *  rotacionX * rotacionY * rotacionZ *scale * triangulo->vertex[0];
				v1 = traslacion *  rotacionX * rotacionY * rotacionZ *scale * triangulo->vertex[1];
				v2 = traslacion *  rotacionX * rotacionY * rotacionZ *scale * triangulo->vertex[2];

				float t;
				Vector coorBaric;
				//Comprobvamos si hay interseccion con el triangulo
				if (intersectionTriangulo(pos, dir, v0, v1, v2, t, coorBaric)) //En caso afirmativo
				{
					Vector rayo = pos + dir*t; //Creamos el rayo
					
					if (zpos < rayo.z || intersecciona == false) { //Comprobamos la Z del triangulo

						//Obtenemos los materiales del triangulo
						SceneMaterial* mat0 = la_escena.GetMaterial(triangulo->material[0]);
						SceneMaterial* mat1 = la_escena.GetMaterial(triangulo->material[1]);
						SceneMaterial* mat2 = la_escena.GetMaterial(triangulo->material[2]);
						Vector color;
						//Interpolamos cada caracteristica del material mediante las coordenadas baricentricas
						Vector colorFinalDifuso = ((mat0->diffuse * coorBaric.x + mat1->diffuse * coorBaric.y + mat2->diffuse * coorBaric.z));
						Vector colorFinalEspecular = ((mat0->specular * coorBaric.x + mat1->specular * coorBaric.y + mat2->specular * coorBaric.z));
						float colorFinalBrillo = ((mat0->shininess * coorBaric.x + mat1->shininess * coorBaric.y + mat2->shininess * coorBaric.z));
						Vector N = ((triangulo->normal[0] * coorBaric.x + triangulo->normal[1] * coorBaric.y + triangulo->normal[2] * coorBaric.z));
						Vector L = (pos - rayo).Normalize();
						Vector R = (N*(N.Dot(L)) * 2) - L;//Vector reflejo, para las llamadas recursivas
						Vector reflexionFinal = ((mat0->reflective * coorBaric.x + mat1->reflective * coorBaric.y + mat2->reflective * coorBaric.z));
						rayo = rayo+N*0.000001; //Offset del rayo
						Vector difuso;
						//Comprobamos si hay textura, y en caso afirmativo, calculamos las coordenadas UV
						// Y cogemos el color de la textura
						if (mat0->texture != "") {
								float U = triangulo->u[0] * coorBaric.x + triangulo->u[1] * coorBaric.y + triangulo->u[2] * coorBaric.z;
								float V = triangulo->v[0] * coorBaric.x + triangulo->v[1] * coorBaric.y + triangulo->v[2] * coorBaric.z;
								if (V < 0) {
									V = V*-1;
								}
								if (U > 0 && V > 0)
									difuso = mat0->GetTextureColor(U, V)/255.0;
						}
						else { //Si no hay textura, cogemos el color difuso del material
							difuso = colorFinalDifuso;
						}
						if (Scene::montecarlo) {
							LuzAmbiental = Vector(0.0, 0.0, 0.0);
							for (int m = 0; m < 32; m++) {
								LuzAmbiental = LuzAmbiental + (MontecarloRay(rayo, m, la_escena,2) / 32);
							}
						}

						//Por cada luz
						for (int l = 0; l < la_escena.GetNumLights(); l++) {
							SceneLight* luzPrueba = la_escena.GetLight(l);
							Vector direcLuz = (luzPrueba->position - rayo); //Direccion de la interseccion hacia la luz
							if (haySombra(rayo + N*0.0001, direcLuz.Normalize(), modulo(direcLuz), la_escena)) { //Calculamos el ShadorRay,
								color = color + LuzAmbiental*0.3; //Si hay sombra, devolvemos el color ambiental
							}
							else { //Si no hay sombra, calculamos iluminacion Phong y realizamos las llamadas recursivas del Raytracing
								color = color + iluminacionPhong(rayo, N, pos, luzPrueba->position, luzPrueba->color, LuzAmbiental, difuso, colorFinalEspecular, colorFinalBrillo);
								color = (color*(Vector(1.0, 1.0, 1.0) - reflexionFinal));
								if(reflexionFinal.x>0.0)
								color = color + (Raytracing(rayo, R, la_escena, rebotes - 1)*reflexionFinal);
							}
						}	
						//Actualizamos profundidad
						intersecciona = true;
						zpos = rayo.z;
						//Devolvemos el color calculado
						respuesta = color;
					}

				}
			}

		}
	}
	return respuesta; //Devolvemos el color
}

// -- Main Functions --
// - CalculatePixel - Returns the Computed Pixel for that screen coordinate
Vector RayTrace::CalculatePixel(int screenX, int screenY)
{
	/*
	-- How to Implement a Ray Tracer --

	This computed pixel will take into account the camera and the scene
	and return a Vector of <Red, Green, Blue>, each component ranging from 0.0 to 1.0

	In order to start working on computing the color of this pixel,
	you will need to cast a ray from your current camera position
	to the image-plane at the given screenX and screenY
	coordinates and figure out how/where this ray intersects with
	the objects in the scene descriptor.
	The Scene Class exposes all of the scene's variables for you
	through its functions such as m_Scene.GetBackground (), m_Scene.GetNumLights (),
	etc. so you will need to use those to learn about the World.

	To determine if your ray intersects with an object in the scene,
	you must calculate where your objects are in 3D-space [using
	the object's scale, rotation, and position is extra credit]
	and mathematically solving for an intersection point corresponding to that object.

	For example, for each of the spheres in the scene, you can use
	the equation of a sphere/ellipsoid to determine whether or not
	your ray from the camera's position to the screen-pixel intersects
	with the object, then from the incident angle to the normal at
	the contact, you can determine the reflected ray, and recursively
	repeat this process capped by a number of iterations (e.g. 10).

	Using the lighting equation & texture to calculate the color at every
	intersection point and adding its fractional amount (determined by the material)
	will get you a final color that returns to the eye at this point.
	*/

	if (screenX == 50 && screenY == 100)
	{
		int kk = 0;
	}
	Scene &la_escena = m_Scene;
	Camera &la_camara = la_escena.GetCamera();
	Vector posicion = la_camara.GetPosition();


	if ((screenX < 0 || screenX > Scene::WINDOW_WIDTH - 1) ||
		(screenY < 0 || screenY > Scene::WINDOW_HEIGHT - 1))
	{
		// Off the screen, return black
		return Vector(0.0f, 0.0f, 0.0f);
	}

	float x = ((screenX * 2.0f) / (float)Scene::WINDOW_WIDTH) - 1.0f;
	float y = ((screenY * 2.0f) / (float)Scene::WINDOW_HEIGHT) - 1.0f;

	//Configuramos la camara segun lo explicado en tutoria
	Camera mycam = la_escena.GetCamera();
	Vector pos = mycam.GetPosition();
	Vector lookat = mycam.GetTarget() - pos;
	Vector up = mycam.GetUp();
	Vector right = lookat.Cross(up);
	up = right.Cross(lookat);
	up.Normalize();
	right.Normalize();
	lookat.Normalize();
	float angle = mycam.GetFOV() * 3.1416f / 180.0f;
	float tangen = tan(angle / 2.0f);
	float aspect = (float)Scene::WINDOW_WIDTH / (float)Scene::WINDOW_HEIGHT;

	int numRebotes = 4; //Numero de rebotes

	//Si escogemos la opcion Supersampling, calculamos el DIR en funcion del offset que apliquemos a las coordenadas XY de los pixeles
	if (Scene::supersample) { 
		float offset = 0.5;
		float xSS1 = (((screenX+offset) * 2.0f) / (float)Scene::WINDOW_WIDTH) - 1.0f;
		float xSS2 = (((screenX-offset) * 2.0f) / (float)Scene::WINDOW_WIDTH) - 1.0f;
		float ySS1 = (((screenY+offset) * 2.0f) / (float)Scene::WINDOW_HEIGHT) - 1.0f;
		float ySS2 = (((screenY-offset) * 2.0f) / (float)Scene::WINDOW_HEIGHT) - 1.0f;
		Vector Dir1 = (lookat + right* aspect * tangen * xSS1 + up * tangen * y).Normalize();
		Vector Dir2 = (lookat + right* aspect * tangen * xSS2 + up * tangen * y).Normalize();
		Vector Dir3 = (lookat + right* aspect * tangen * x + up * tangen * ySS1).Normalize();
		Vector Dir4 = (lookat + right* aspect * tangen * x + up * tangen * ySS2).Normalize();
		return (Raytracing(pos, Dir1, la_escena, numRebotes) + Raytracing(pos, Dir2, la_escena, numRebotes) + Raytracing(pos, Dir3, la_escena, numRebotes) + Raytracing(pos, Dir4, la_escena, numRebotes)) / 4.0;
	}
	else {
		Vector dir;
		dir = lookat + right* aspect * tangen * x + up * tangen * y;
		dir.Normalize();
		return Raytracing(pos, dir, la_escena, numRebotes);
	}

	// Until this function is implemented, return white
	//return Vector (1.0f, 1.0f, 1.0f);
}

//https://www.ics.uci.edu/~gopi/CS211B/RayTracing%20tutorial.pdf