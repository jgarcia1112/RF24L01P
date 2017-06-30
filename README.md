# arduino
Miercoles 21 de Junio 2017:

  Nuevas correcciones, y mejoras.
  
    - Ya se puede usar en modo estrella 1:6, tan sencillo como poner uno en modo Rx y los demas en modo Tx.
    
    - Cada dispotivio Tx es un canal logico, asi que solamente se inicializa como :  channel1, channel2, ... channel6.
    
    - El receptor se puede iniciar (startRx) y detener (standby) en cualquier momento segun lo desees.
    
    - Todo esta configurado hasta ahora, para que funcione con los valores que trae por default, asi que basicamente es de instalar y usar.
    
    Seguire trabajando y programando en este maravilloso mundo de Arduino para hacer aportaciones, compartir y mejorar.
    
    
    
    -------------------------------------------------------------------------------------------------------------

Domingo 18 Junio 1027:

  - He actualizado mi codigo fuente de este modulo NRF24L01+
  
  No esta completo, aun lo sigo depurando, sin embargo esta a tu disposicion si te hes de utilidad, y sin ningun tipo de garantia.
  
  Estos archivos los ire cambiando sin previo aviso conforme lo vaya mejorando.
  
  
  - Actualmente, se puede usar en modo uno a uno , utilizando los ajustes que trae por defecto el modulo.
  - Esta hecho de modo que funciona con el ESB activado, cargas dinamicas y auto-confirmacion.
  
  Espero te sea de utilidad y bienvenidas las modificaciones!!.. 
  
  -------------------------------------------------------------------------------------------------------------

Coleccion de codigo fuente para arduino. De Tabasco, Mexico, para el Mundo

Estoy haciendo un proyecto que tengo en mente de telemetria. 
Soy nuevo en el mundo de Arduino y me ha parecido fascinante.

Recientemente compre una tarjeta Arduino Uno y un par de sensores, y este modulo de Radio Frecuencia de Nordic Semiconductors.

En el proyecto, pretendo tomar lectura de un medidor de distancia, un barometro, e intensidad luminosa, todo eso es para
tener los datos dentro de un contenedor de agua (tinaco) y conocer sus condiciones para poder indicar su nivel,  y saber 
cuando activar un motor para que se llene el tinaco, y saber que tan turbia esta el agua, y cosas asi.

Arduino me parece una plataforma interesante, pues es posible integrar el hardware con el software de una manera que hace 
algunos años simplemente era algo imposible para mi.  Conozco de electronica, de computacion de programacion y aqui se une
todo en un solo concepto, y lo demas.. es imaginacion!.

Me gusta contribuir, por eso comparto esto, que quizas ya existe, quizas hay algo mucho mejor, pero tambien quizas (como 
fue en mi caso) de aqui nazcan nuevas ideas, proyectos, conceptos que se puedan aplicar y se puedan mejorar en muchos de los
muchos dispositivos que a diario nos rodean.

Posteriormente publicare los ejemplos en Arduino usando esta libreria. y tratare en lo posible, estar alimentando este espacio, 
sobre todo, un espacio para los que se nos hace dificil el ingles!.

Modificacion 25 de Octubre 2015. 
Se le han agregado funciones que permiten usarlo en modo multiceiver 1:6 y esta en etapa de pruebas, las cuales han sido muy satisfactorias.
Pronto estare probando con los sensores que usare en mi proyecto y seguire compartiando siempre que pueda, el codigo y los ejemplos para continuar con su desarrollo.
