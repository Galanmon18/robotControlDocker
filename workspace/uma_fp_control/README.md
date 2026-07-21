Estructura de este archivo:

- src: Archivo principal de la aplicación
    - Se encarga unicamente de coordinar los servicios
- services: Archivo de servicios
    - Son funcionalidades por si mismas como el control que dependen de otras funcionalidades
- dependecies: Archivo de dependencias
    - Son funcionalidades que no dependen de otras funcionalidades, solo de si mismas
- mocks: Archivo de simulaciones
- launch: Archivo de lanzamiento
    - Se encarga de lanzar y crear los servicios y coordinadores
- config: Archivo de con las transformadas de claibracion