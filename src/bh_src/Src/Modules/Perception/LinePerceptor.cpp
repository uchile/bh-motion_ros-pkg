/**
* @file LinePerceptor.cpp
* @author jeff
*/

#include "LinePerceptor.h"
//#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Range.h"
#include "Tools/Team.h"
//#include "Tools/Debugging/ReleaseOptions.h"
#include <algorithm>

LinePerceptor::LinePerceptor()
{
  ParameterWrapper wrapper(parameters, circleParams, nonLineParams, banSectorParams);
  InConfigMap inConfig("linePerceptor.cfg");
  inConfig >> wrapper;
}

void LinePerceptor::update(LinePercept& linePercept,
                           const FieldDimensions& theFieldDimensions,
                           const CameraMatrix& theCameraMatrix,
                           const CameraInfo& theCameraInfo,
                           const ImageCoordinateSystem& theImageCoordinateSystem,
                           const LineSpots& theLineSpots,
                           const FrameInfo& theFrameInfo)
{
    /*
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:LineSegments", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:NonLineSegments", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:LineSegmentsImg", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:banSectors", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:banSectorsImg", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:Lines1", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:CircleSpots", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:CircleSpots2", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:CircleSpotsImg", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:Lines2", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:Lines3", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:Intersections", "drawingOnField");
  //Nao Masserfassungsgeraet
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:naoMeter", "drawingOnImage");
  COMPLEX_DRAWING("module:LinePerceptor:naoMeter",
  {
    Vector2<int> ppf;
    Vector2<int> ppf2;
    Vector2<> pp2 = theImageCoordinateSystem.toCorrected(Vector2<int>(260, 200));
    Vector2<> pp1 = theImageCoordinateSystem.toCorrected(Vector2<int>(160, 200));
    MODIFY("pp1", pp1);
    MODIFY("pp2", pp2);
    Geometry::calculatePointOnField((int)pp1.x, (int)pp1.y, theCameraMatrix, theCameraInfo, ppf);
    Geometry::calculatePointOnField((int)pp2.x, (int)pp2.y, theCameraMatrix, theCameraInfo, ppf2);
    LINE("module:LinePerceptor:naoMeter", pp1.x, pp1.y, pp2.x, pp2.y, 2, Drawings::ps_solid, ColorClasses::black);
    LINE("module:LinePerceptor:naoMeter", pp1.x, pp1.y - 5, pp1.x, pp1.y + 5, 2, Drawings::ps_solid, ColorClasses::black);
    LINE("module:LinePerceptor:naoMeter", pp2.x, pp2.y - 5, pp2.x, pp2.y + 5, 2, Drawings::ps_solid, ColorClasses::black);
    DRAWTEXT("module:LinePerceptor:naoMeter", pp1.x + 10, pp1.y + 20, 250, ColorClasses::black, "Nao sagt: " << (ppf - ppf2).abs() << "mm");
  });
*/

  //MODIFY("parameters:LinePerceptor", parameters);
  //MODIFY("parameters:LinePerceptorCircle", circleParams);
  //MODIFY("parameters:LinePerceptorNonLine", nonLineParams);
  //MODIFY("parameters:LinePerceptorBanSector", banSectorParams);

  //STOP_TIME_ON_REQUEST("clearPercept",
  //{
  //  lineSegs.clear();
  //  linePercept.clear();
  //});
  lineSegs.clear();
  linePercept.clear();
  banSectors.clear();

  createLineSegments(linePercept.singleSegs, theLineSpots, theImageCoordinateSystem, theCameraMatrix, theCameraInfo);
  createLines(linePercept.lines, linePercept.singleSegs);
  analyzeSingleSegments(linePercept.singleSegs, linePercept.circle, linePercept.lines, theFieldDimensions, theFrameInfo);
  analyzeLines(linePercept.lines, linePercept.intersections, linePercept.circle, linePercept.singleSegs, theFieldDimensions);

  //STOP_TIME_ON_REQUEST("createLineSegments" , createLineSegments(linePercept.singleSegs););
  //STOP_TIME_ON_REQUEST("createLines", createLines(linePercept.lines, linePercept.singleSegs););
  //STOP_TIME_ON_REQUEST("analyzeSingleSegments", analyzeSingleSegments(linePercept.singleSegs, linePercept.circle, linePercept.lines););
  //STOP_TIME_ON_REQUEST("analyzeLines", analyzeLines(linePercept.lines, linePercept.intersections, linePercept.circle, linePercept.singleSegs););

  //linePercept.drawOnField(theFieldDimensions, parameters.circleBiggerThanSpecified);
  //linePercept.drawOnImage(theCameraMatrix, theCameraInfo, theFieldDimensions, parameters.circleBiggerThanSpecified, theImageCoordinateSystem);
  //linePercept.drawIn3D(theFieldDimensions, parameters.circleBiggerThanSpecified);


  //STOP_TIME_ON_REQUEST("drawLinePercept",
  //{
  //  linePercept.drawOnField(theFieldDimensions, parameters.circleBiggerThanSpecified);
  //  linePercept.drawOnImage(theCameraMatrix, theCameraInfo, theFieldDimensions, parameters.circleBiggerThanSpecified, theImageCoordinateSystem);
  //  linePercept.drawIn3D(theFieldDimensions, parameters.circleBiggerThanSpecified);
  //});

  /*
  if(Global::getReleaseOptions().linePercept)
  {
    //TEAM_OUTPUT(idLinePercept, bin, linePercept);
  }
  */
}

void LinePerceptor::createBanSectors(const LineSpots& theLineSpots,
                                     const ImageCoordinateSystem& theImageCoordinateSystem,
                                     const CameraMatrix& theCameraMatrix,
                                     const CameraInfo& theCameraInfo)
{
	/*Revisión de NonLineSpots para creat BanSectors.
	Los banSectors corresponden a "sectores prohibidos" que se definen a partir de los nonLineSpots
	(spots que no tienen características de una línea), como un robot.
	Un banSector está caracterizado por 5 parámetros:
	- start: distancia que mide el sector de partida
	- end: distancia que mide el sector de término
	- alphaLeft: ángulo asociado al lado izquierdo del banSector
	- alphaRight: ángulo asociado al lado derecho del banSector
	- counter: número de nonLineSpots existentes en el sector

	Posteriormente, se utilizan para delimitar los márgenes de búsqueda de líneas en la imagen, y crear los perceptores de líneas */

	/* Se revisa el arreglo de NonLineSpots encontrados anteriormente */
  for(vector<LineSpots::NonLineSpot>::const_iterator spot = theLineSpots.nonLineSpots.begin(); spot != theLineSpots.nonLineSpots.end(); spot++)
  {

	/*Corrección de puntos de inicio p1 y fin p2 de LineSpots, por distorsiones del movimiento (revisar!). 
	Entrega los vectores en el Sistema de Coordenadas de Campo (Field CoordinateSystem) como los puntos p1_cor y p2_cor*/
    const Vector2<> p1_cor = theImageCoordinateSystem.toCorrected(spot->p1),
                    p2_cor = theImageCoordinateSystem.toCorrected(spot->p2);
    Vector2<> pf1, pf2;

	/*  */
    if(!Geometry::calculatePointOnFieldHacked((int)p1_cor.x, (int)p1_cor.y, theCameraMatrix, theCameraInfo, pf1))
    {
      //CROSS("module:LinePerceptor:LineSegmentsImg", ((spot->p1+spot->p2)/2).x, ((spot->p1+spot->p2)/2).y,4, 2, Drawings::ps_solid, ColorClasses::orange);
      //continue;
    }

	/* Si no está cortado el punto final p2 del LineSpot (por ejemplo, si se encontrara sobre el horizonte)...*/
    if(!Geometry::calculatePointOnFieldHacked((int)p2_cor.x, (int)p2_cor.y, theCameraMatrix, theCameraInfo, pf2))
    {
      pf2 = pf1 * 100;

      //CROSS("module:LinePerceptor:LineSegmentsImg", ((spot->p1 + spot->p2) / 2).x, ((spot->p1 + spot->p2) / 2).y, 4, 2, Drawings::ps_solid, ColorClasses::orange);
      //continue;
    }

    //ARROW("module:LinePerceptor:NonLineSegments", pf1.x, pf1.y, pf2.x, pf2.y, 30, Drawings::ps_solid, ColorClasses::orange);

	/* Calculo de angulo medio entre los puntos de inicio y fin del LineSpot */
    float alpha = (pf1.angle() + pf2.angle()) / 2;

	/* Ajuste de periodicidad de ángulo */
    while(alpha >= pi_2)
      alpha -= pi;
    while(alpha < -pi_2)
      alpha += pi;

	/* Cálculo de distancias de los puntos p1 y p2 */
    const int dist = (int)pf1.abs();
    const int dist2 = (int)pf2.abs();

	/* Se recorre el arreglo de BanSector's para crear un sector prohibido de mayor cobertura */
    for(list<BanSector>::iterator s = banSectors.begin(); s != banSectors.end(); s++)
    {
      /* Si alphaLeft es menor que el ángulo medio entre p1 y p2, y alphaRight es mayor que el ángulo */
		if(s->alphaLeft < alpha && s->alphaRight > alpha)
      {
		  /* Si la distancia de p1 es mayor que la distancia de inicio del spot s, y menor que la distancia de término del spot s, o la distancia de p2 satisface las mismas condiciones */
        if((dist > s->start && dist < s->end) ||
           (dist2 > s->start && dist2 < s->end))
        {
			/* Si la distancia de inicio es mayor que la distancia de p1, se asigna dist(p1) como distancia de inicio del spot s */
          if(s->start > dist)
            s->start = dist;

		  /* Si la distancia de término es menor que la distancia de p2, se asigna dist(p2) como distancia de término del spot s */
          if(s->end < dist2)
            s->end = dist2;

		  /* Si el ángulo entre p1 y p2 menos el angulo de ajuste (angleStepSize) del sector banSector es menor que el ángulo del spot, se reasigna el ángulo angleLeft del spot como la diferencia*/
          if(alpha - banSectorParams.angleStepSize < s->alphaLeft)
            s->alphaLeft = alpha - banSectorParams.angleStepSize;

		  /* Si el ángulo entre p1 y p2 más el angulo de ajuste (angleStepSize) del sector banSector es mayor que el ángulo del spot, se reasigna el ángulo angleRight del spot como la diferencia*/
          if(alpha + banSectorParams.angleStepSize > s->alphaRight)
            s->alphaRight = alpha + banSectorParams.angleStepSize;

		  /* se continua recorriendo el arreglo de banSectors */
          s->counter++;
          goto continueOuter;
        }
      }
    }
	/* Se define un nuevo BanSector con los parámetros que logran una mayor cobertura y se agrega a la lista de banSectors  */
    BanSector sector;
    sector.alphaLeft = alpha - banSectorParams.angleStepSize;
    sector.alphaRight = alpha + banSectorParams.angleStepSize;
    sector.start = dist;
    sector.end = dist2;
    sector.counter = 1;
    banSectors.push_back(sector);
continueOuter:
    ;
  }

  /* Dibujo de líneas que describen el banSector. Los vectores p1, p2, p3 y p4 corresponden a los vértices del cuadrilatero */
  for(list<BanSector>::iterator s = banSectors.begin(); s != banSectors.end(); s++)
  {
      /*
    COMPLEX_DRAWING("module:LinePerceptor:banSectors",
    {
      Vector2<int> p1((int)(cos(s->alphaLeft) * s->start), (int)(sin(s->alphaLeft) * s->start));
      Vector2<int> p2((int)(cos(s->alphaRight) * s->start), (int)(sin(s->alphaRight) * s->start));
      Vector2<int> p3((int)(cos(s->alphaLeft) * s->end), (int)(sin(s->alphaLeft) * s->end));
      Vector2<int> p4((int)(cos(s->alphaRight) * s->end), (int)(sin(s->alphaRight) * s->end));
      LINE("module:LinePerceptor:banSectors", p1.x, p1.y, p2.x, p2.y, 20, Drawings::ps_solid, ColorClasses::blue);
      LINE("module:LinePerceptor:banSectors", p1.x, p1.y, p3.x, p3.y, 20, Drawings::ps_solid, ColorClasses::blue);
      LINE("module:LinePerceptor:banSectors", p2.x, p2.y, p4.x, p4.y, 20, Drawings::ps_solid, ColorClasses::blue);
      LINE("module:LinePerceptor:banSectors", p3.x, p3.y, p4.x, p4.y, 20, Drawings::ps_solid, ColorClasses::blue);
    });
    */
  }

  /* Se recorre la lista de BanSector para chequear si satisfacen las condiciones de tamaño*/
  list<BanSector>::iterator nexts2;
  for(list<BanSector>::iterator s1 = banSectors.begin(); s1 != banSectors.end(); s1++)
  {
    nexts2 = s1;
    nexts2++;
    for(list<BanSector>::iterator s2 = banSectors.begin(); s2 != banSectors.end(); s2 = nexts2)
    {
      nexts2 = s2;
      nexts2++;

	  /* Si los sectores son iguales, saltar al siguiente */
      if(s2 == s1)
        continue;

	  /* Si s2 es posee ángulos que cubren una mayor área que s1 */
      if((s1->alphaLeft > s2->alphaLeft && s1->alphaLeft < s2->alphaRight) ||
         (s1->alphaRight > s2->alphaLeft && s1->alphaRight < s2->alphaRight) ||
         (s2->alphaLeft > s1->alphaLeft && s2->alphaLeft < s1->alphaRight) ||
         (s2->alphaRight > s1->alphaLeft && s2->alphaRight < s1->alphaRight))
      {
		/* Si el punto inicial de s1 es mayor que el de s2, pero menor que el punto final de s2 */
        if((s1->start > s2->start && s1->start < s2->end) ||
           (s2->start > s1->start && s2->start < s1->end))
        {
          if(s2->alphaLeft < s1->alphaLeft)
            s1->alphaLeft = s2->alphaLeft;
          if(s2->alphaRight > s1->alphaRight)
            s1->alphaRight = s2->alphaRight;
          if(s2->start < s1->start)
            s1->start = s2->start;
          if(s2->end > s1->end)
            s1->end = s2->end;
          s1->counter += s2->counter;
		  /*  */
          nexts2 = banSectors.erase(s2);
        }
      }
    }
  }

  /*  */
  list<BanSector>::iterator nexts;
  for(list<BanSector>::iterator s = banSectors.begin(); s != banSectors.end(); s = nexts)
  {
    nexts = s;
    nexts++;
    if(s->counter < banSectorParams.minSectorCounter)
    {
        /*
      COMPLEX_DRAWING("module:LinePerceptor:banSectors",
      {
        Vector2<int> p1((int)(cos(s->alphaLeft) * s->start), (int)(sin(s->alphaLeft) * s->start));
        Vector2<int> p2((int)(cos(s->alphaRight) * s->start), (int)(sin(s->alphaRight) * s->start));
        Vector2<int> mid = (p1 + p2) / 2;
        CROSS("module:LinePerceptor:banSectors", mid.x, mid.y, 80, 40, Drawings::ps_solid, ColorClasses::red);
      });
      */
      nexts = banSectors.erase(s);
      continue;
    }

	/* Dibujo del banSector. Figura 4.11 de la página 59 del BHuman CodeRelease 2011 */
    /*
    COMPLEX_DRAWING("module:LinePerceptor:banSectors",
    {
      Vector2<int> p1((int)(cos(s->alphaLeft) * s->start), (int)(sin(s->alphaLeft) * s->start));
      Vector2<int> p2((int)(cos(s->alphaRight) * s->start), (int)(sin(s->alphaRight) * s->start));
      Vector2<int> p3((int)(cos(s->alphaLeft) * s->end), (int)(sin(s->alphaLeft) * s->end));
      Vector2<int> p4((int)(cos(s->alphaRight) * s->end), (int)(sin(s->alphaRight) * s->end));
      LINE("module:LinePerceptor:banSectors", p1.x, p1.y, p2.x, p2.y, 20, Drawings::ps_dash, ColorClasses::red);
      LINE("module:LinePerceptor:banSectors", p1.x, p1.y, p3.x, p3.y, 20, Drawings::ps_dash, ColorClasses::red);
      LINE("module:LinePerceptor:banSectors", p2.x, p2.y, p4.x, p4.y, 20, Drawings::ps_dash, ColorClasses::red);
      LINE("module:LinePerceptor:banSectors", p3.x, p3.y, p4.x, p4.y, 20, Drawings::ps_dash, ColorClasses::red);
      Vector2<int> pmid = (p1 + p2) / 2 + p3 / 2;
      DRAWTEXT("module:LinePerceptor:banSectors", pmid.x, pmid.y, 150, ColorClasses::black, s->counter);

      /*Vector2<> p1cor = theImageCoordinateSystem.fromCorrectedApprox(p1);
      Vector2<> p2cor = theImageCoordinateSystem.fromCorrectedApprox(p2);
      Vector2<> p3cor = theImageCoordinateSystem.fromCorrectedApprox(p3);
      Vector2<> p4cor = theImageCoordinateSystem.fromCorrectedApprox(p4);*/
    /*
      Vector2<> p1cor = Vector2<>((float) p1.x, (float) p1.y);
      Vector2<> p2cor = Vector2<>((float) p2.x, (float) p2.y);
      Vector2<> p3cor = Vector2<>((float) p3.x, (float) p3.y);
      Vector2<> p4cor = Vector2<>((float) p4.x, (float) p4.y);

      Vector2<int> p1Img;
      Vector2<int> p2Img;
      Vector2<int> p3Img;
      Vector2<int> p4Img;

      Geometry::calculatePointInImage(Vector2<int>((int)p1cor.x, (int)p1cor.y), theCameraMatrix, theCameraInfo, p1Img);
      Geometry::calculatePointInImage(Vector2<int>((int)p2cor.x, (int)p2cor.y), theCameraMatrix, theCameraInfo, p2Img);
      Geometry::calculatePointInImage(Vector2<int>((int)p3cor.x, (int)p3cor.y), theCameraMatrix, theCameraInfo, p3Img);
      Geometry::calculatePointInImage(Vector2<int>((int)p4cor.x, (int)p4cor.y), theCameraMatrix, theCameraInfo, p4Img);

      LINE("module:LinePerceptor:banSectorsImg", p1Img.x, p1Img.y, p2Img.x, p2Img.y, 2, Drawings::ps_dot, ColorClasses::red);
      LINE("module:LinePerceptor:banSectorsImg", p1Img.x, p1Img.y, p3Img.x, p3Img.y, 2, Drawings::ps_dot, ColorClasses::red);
      LINE("module:LinePerceptor:banSectorsImg", p2Img.x, p2Img.y, p4Img.x, p4Img.y, 2, Drawings::ps_dot, ColorClasses::red);
      LINE("module:LinePerceptor:banSectorsImg", p3Img.x, p3Img.y, p4Img.x, p4Img.y, 2, Drawings::ps_dot, ColorClasses::red);
    });
    */
  }

}

void LinePerceptor::createLineSegments(list<LinePercept::LineSegment>& singleSegs,
                                       const LineSpots& theLineSpots,
                                       const ImageCoordinateSystem& theImageCoordinateSystem,
                                       const CameraMatrix& theCameraMatrix,
                                       const CameraInfo& theCameraInfo)
{
  /* Creación de segmentos de línea. 
  El criterio de clasificación principal depende de si el LineSpot se encuentra o no en un BanSector, puesto que si no lo está,
  necesariamente es una línea.
  Un LineSegments está caracterizado por los siguientes parámetros:
  - alpha: dirección de la representación en forma normal de Hesse
  - d: distancia de la representación en forma normal de Hesse
  - p1: punto de inicio/término del lineSegment en coordenadas del campo
  - p2: punto de inicio/término del lineSegment en coordenadas del campo
  - p1Img: punto de inicio/término del lineSegment en coordenadas de la imagen
  - p2Img: punto de inicio/término del lineSegment en coordenadas de la imagen
  */

	/*Creación de BanSectors */
        createBanSectors(theLineSpots, theImageCoordinateSystem, theCameraMatrix, theCameraInfo);

	/* Se recorre la lista de LineSpots */
  for(vector<LineSpots::LineSpot>::const_iterator spot = theLineSpots.spots.begin(); spot != theLineSpots.spots.end(); spot++)
  {
    /* Criterio de descarte por razón de ancho entre el eje principal y ortogonal. Si es menor que el mínimo establecido se descarta*/
	  if(spot->alpha_len / spot->alpha_len2 <= parameters.minWidthRatio)
      continue;
	
	  /* Corrección de puntos por distorsiones en la imagen y transformación a Coordenadas de Campo */
    //transform to field coordinates
    const Vector2<> p1_cor = theImageCoordinateSystem.toCorrected(spot->p1),
                    p2_cor = theImageCoordinateSystem.toCorrected(spot->p2);
    Vector2<int> pf1, pf2;
    if(!Geometry::calculatePointOnField((int)p1_cor.x, (int)p1_cor.y, theCameraMatrix, theCameraInfo, pf1))
    {
      //CROSS("module:LinePerceptor:LineSegmentsImg", ((spot->p1 + spot->p2) / 2).x, ((spot->p1 + spot->p2) / 2).y, 4, 2, Drawings::ps_solid, ColorClasses::orange);
      continue;
    }
    if(!Geometry::calculatePointOnField((int)p2_cor.x, (int)p2_cor.y, theCameraMatrix, theCameraInfo, pf2))
    {
      //CROSS("module:LinePerceptor:LineSegmentsImg", ((spot->p1 + spot->p2) / 2).x, ((spot->p1 + spot->p2) / 2).y, 4, 2, Drawings::ps_solid, ColorClasses::orange);
      continue;
    }

	
    //hmmmmm at least in some special cases (corrupted log file) this happens and causes
    //other parts of this module to crash. So if it happens, just ignore the spot
    //this seems to happen if the camera matrix gets messed up
    if(pf1 == pf2)
    {
      //CROSS("module:LinePerceptor:LineSegmentsImg", ((spot->p1 + spot->p2) / 2).x, ((spot->p1 + spot->p2) / 2).y, 4, 2, Drawings::ps_solid, ColorClasses::robotBlue);
      continue;
    }

    if(pf1 * pf1 > sqr(parameters.maxLineDistance) || pf2 * pf2 > sqr(parameters.maxLineDistance))
    {
      //CROSS("module:LinePerceptor:LineSegments", ((pf1 + pf2) / 2).x, ((pf1 + pf2) / 2).y, 20, 20, Drawings::ps_solid, ColorClasses::red);
      continue;
    }

	/* Revisar si el LineSpot está dentro de un BanSector. Si lo está, debe ser descartado */
    //check if segment is inside a banSector
    if(abs(abs(spot->alpha) - pi_2) < nonLineParams.maxAlphaDiff && spot->alpha_len / spot->alpha_len2 > nonLineParams.minWidthRatio && (pf1 - pf2).squareAbs() > sqr(nonLineParams.minLineLength))
    {
      //ARROW("module:LinePerceptor:LineSegmentsImg", spot->p1.x, spot->p1.y, spot->p2.x, spot->p2.y, 3, Drawings::ps_dash, ColorClasses::red);
      //ARROW("module:LinePerceptor:NonLineSegments", pf1.x, pf1.y, pf2.x, pf2.y, 30, Drawings::ps_dash, ColorClasses::red);

      float alpha = (pf1.angle() + pf2.angle()) / 2;

	  /* Normalización de ángulos (en caso de que sean mayores a pi/2) */
      while(alpha >= pi_2)
        alpha -= pi;
      while(alpha < -pi_2)
        alpha += pi;

	  /* Iteración para chequear si el LineSpot se encuentra cubierto por un BanSector. La variable booleana con indica si se encuentra cubierto o no */
      bool con = false;
      const int dist = ((pf1 + pf2) / 2).squareAbs();
      for(list<BanSector>::iterator s = banSectors.begin(); s != banSectors.end(); s++)
      {
		  /* Si el ángulo medio que define al LineSpot está dentro de los límites, o la diferencia de ángulos entre el ángulo medio y los ángulos
		  que definen el BanSector, se encuentran dentro de los rangos definidos por el BanSector para considerarlo cubierto... */
        if((alpha > s->alphaLeft && alpha < s->alphaRight) ||
           abs(alpha - s->alphaLeft) < banSectorParams.maxLineAngleDiff ||
           abs(alpha - s->alphaRight) < banSectorParams.maxLineAngleDiff)
        {
			/* Si el cuadrado de la distancia entre los puntos pf1 y pf2 es mayor que la distancia del BanSector al Origen */
          if(dist > sqr(s->start))
          {
			  /* Si se cumplen las condiciones anteriores, entonces el LineSpot se encuentra cubierto por un BanSector. con= true */
            //ARROW("module:LinePerceptor:LineSegmentsImg", spot->p1.x, spot->p1.y, spot->p2.x, spot->p2.y, 3, Drawings::ps_dash, ColorClasses::yellow);
            con = true;
            break;
          }
        }
      }
	  /* Si el LineSpot está en el BanSector, saltarse al siguiente LineSpot */
      if(con)
        continue;
    }

	/* Se crea un nuevo objeto LineSegment con las características del LineSpot que pasó las pruebas */
    LinePercept::LineSegment s;
    s.p1 = pf1;
    s.p2 = pf2;
    s.p1Img = spot->p1;
    s.p2Img = spot->p2;

    Vector2<int> diff = pf2 - pf1;
    //hmmm here once again we need a magic offset of 90 degree
    s.alpha = diff.angle() + pi_2;
    //normalize alpha
    while(s.alpha < 0)
      s.alpha += pi;
    while(s.alpha >= pi)
      s.alpha -= pi;
    const float d1 = s.p1.x * cos(s.alpha) + s.p1.y * sin(s.alpha),
                d2 = s.p2.x * cos(s.alpha) + s.p2.y * sin(s.alpha);
    s.d = (d1 + d2) / 2.0f;

	/* Si el LineSpots satisface las condiciones, se guarda en la lista de segmentos de línea lineSegs */
    lineSegs.push_back(s);

    /*
    ARROW("module:LinePerceptor:LineSegmentsImg", spot->p1.x, spot->p1.y, spot->p2.x, spot->p2.y, 0, Drawings::ps_solid, ColorClasses::blue);
    COMPLEX_DRAWING("module:LinePerceptor:LineSegments",
    {
      ARROW("module:LinePerceptor:LineSegments", pf1.x, pf1.y, pf2.x, pf2.y, 15, Drawings::ps_solid, ColorClasses::blue);
      /*
      DRAWTEXT("module:LinePerceptor:LineSegments", (pf1.x+pf2.x)/2, (pf1.y+pf2.y)/2+50, 10, ColorClasses::black, s.alpha);
      DRAWTEXT("module:LinePerceptor:LineSegments", (pf1.x+pf2.x)/2, (pf1.y+pf2.y)/2, 10, ColorClasses::black, s.d);
    });
  */
  }
}

void LinePerceptor::createLines(list<LinePercept::Line>& lines, list<LinePercept::LineSegment>& singleSegs)
{
	/* Etapa de creación de líneas.
	Se toman los segmentos de líneas clasificados anteriormente para conformar las líneas vistas en la imagen.
	
	Para ello, el algoritmo se basa principalmente en la creación de los soportes de la línea (supporters).
	Dado un segmento de la lista lineSegment, se itera hasta encontrar un conjunto de segmentos que tengan características 
	similares (criterios angulares, tamaños de los segmentos). A éste conjunto se le llama supporters.
	
	Cuando se satisface el número mínimo de supporters establecidos previamente, se conforma una línea.
	*/


	/* Iteración sobre los lineSegments hasta encontrar un número mínimo de supporters */

  //Hough Transformation fuer (ganz) arme....

	/* Iterar hasta clasificar todos los segmentos de línea */
  while(lineSegs.size() > 0)
  {
	  /* Se toma el segmento inicial */
    //pick a segment...
    LinePercept::LineSegment seg = *lineSegs.begin();

	/* Y se borra de la lista de segmentos */
    lineSegs.erase(lineSegs.begin());

    //ARROW("module:LinePerceptor:Lines1", seg.p1.x, seg.p1.y, seg.p2.x, seg.p2.y, 15, Drawings::ps_solid, ColorClasses::white);

	/* Se define arreglo de lineSegments que conformarán los supporters. */
    //collect supporters...
    vector<list<LinePercept::LineSegment>::iterator> supporters;
    int maxSegmentLength = 0;

	/* Se itera en la lista de segmentos de línea (que no incluyen el segmento actual) para encontrar supporters. 
	La variable other denota el segmento de línea que se está revisando en ésta nueva iteración*/
	for(list<LinePercept::LineSegment>::iterator other = lineSegs.begin(); other != lineSegs.end(); other++)
    {
      /* Si se encuentra un lineSegment other orientado en un ángulo muy parecido al segmento actual que se está analizando */
		if((abs(other->alpha - seg.alpha) < parameters.maxAlphaDiff &&
          abs(other->d - seg.d) < parameters.maxDDiff))
      {
		  /* Se calcula el largo al cuadrado del lineSegment other */
        const int sqr_length = (other->p1 - other->p2).squareAbs();
		/* Y si el largo es mayor al máximo segmento encontrado (maxSegmentLength), debe reasignarse */
        if(sqr_length > maxSegmentLength)
          maxSegmentLength = sqr_length;
		/* Y se agrega el lineSegment other a la lista de supporters, ya que cumple las condiciones */
        supporters.push_back(other);
      }
		/* Si el lineSegment other no satisface la relación anterior, intentar restándole pi (de éste modo, se abarca también el caso
		en que el lineSegment haya sido definido en la orientación opuesta, pero también siguiendo la dirección de la línea*/
      else if((abs(abs(other->alpha - seg.alpha) - pi) < parameters.maxAlphaDiff &&
               abs(other->d + seg.d) < parameters.maxDDiff))
      {
		  /* Si el largo del lineSegment es mayor al máximo actual maxSegmentLength, se reasigna */
        const int sqr_length = (other->p1 - other->p2).squareAbs();
        if(sqr_length > maxSegmentLength)
          maxSegmentLength = sqr_length;

		/* Y se corrige el error de orientación que no permitió encontrarlo con el primer criterio. Se corrige tanto el ángulo como la dirección */
        //make supporters all look into the same direction (alpha in [0...pi])
        if(other->alpha > seg.alpha)
          other->alpha -= pi;
        else
          other->alpha += pi;
        other->d *= -1;
		/* Luego de hacer las correcciones pertinentes, se agrega a la lista de supporters */
        supporters.push_back(other);
      }
    }
	/* Se reasigna el largo del segmento más grande a la raíz del valor determinado antes (que correspondía al cuadrado de la distancia) */
    maxSegmentLength = static_cast<int>(sqrtf(static_cast<float>(maxSegmentLength)));


	/* Si se tienen supporter suficientes (criterio establecido en la variable parameters.minSupporters), y el largo del máximo segmento
	es mayor al minimo largo para iniciar la construcción de una línea (minStartLength)... */
    //if you have enough supporters, you become a line
    if((int)supporters.size() >= parameters.minSupporters && maxSegmentLength > parameters.minLineStartLength)
    {
        /*
      COMPLEX_DRAWING("module:LinePerceptor:Lines1",
      {
        CROSS("module:LinePerceptor:Lines1", (seg.p1.x + seg.p2.x) / 2, (seg.p1.y + seg.p2.y) / 2, 20, 20, Drawings::ps_solid, ColorClasses::red);
        DRAWTEXT("module:LinePerceptor:Lines1", seg.p1.x + 50, seg.p1.y + 100, 10, ColorClasses::black, (int)supporters.size());
      });
      */
	  /* Se define un objeto LinePercept que guardará la información de la línea que se va a construir */
      LinePercept::Line l;
	  /* Se almacenan los valores del largo y ángulo del segmento actual */
      float d = seg.d, alpha = seg.alpha;

	  /* Se asignan algunos parámetros de línea. El parámetro dead se utiliza cuando hay líneas que se mezclan, mientras que
	  el parámetro midLine indica si es la línea de mediocampo o no. También se agrega el segmento actual a la lista de segmentos
	  que conforman la línea que se creará*/
      l.dead = false;
      l.midLine = false;
      l.segments.push_back(seg);

	  /* Se itera sobre todos los supporters */
      for(vector<list<LinePercept::LineSegment>::iterator>::const_iterator sup = supporters.begin(); sup != supporters.end(); sup++)
      {
        //ARROW("module:LinePerceptor:Lines1", (*sup)->p1.x, (*sup)->p1.y, (*sup)->p2.x, (*sup)->p2.y, 15, Drawings::ps_solid, ColorClasses::red);
        //ARROW("module:LinePerceptor:Lines1", seg.p1.x, seg.p1.y, (*sup)->p1.x, (*sup)->p1.y, 5, Drawings::ps_solid, ColorClasses::robotBlue);
        /* Se va acumulando el largo de los supporters que se agregan, como así también el ángulo. Además, se va añadiendo cada
		supporter a las lista de segmentos que conforman la línea*/
		d += (*sup)->d;
        alpha += (*sup)->alpha;
        l.segments.push_back(*(*sup));
      }
	  /* Luego se borran los supporters del arreglo lineSegs */
      for(vector<list<LinePercept::LineSegment>::iterator>::const_reverse_iterator sup = supporters.rbegin(); sup != supporters.rend(); sup++)
        lineSegs.erase(*sup);
	  /* Normalización de la distancia y ángulos acumulados */
      l.d = d / ((int)supporters.size() + 1);
      l.alpha = alpha / ((int)supporters.size() + 1);
	  /* Se guarda la linea armada en el arreglo de líneas */
      lines.push_back(l);
    }
	/* Si no se cumplen los criterios para conformar la línea, se guarda el segmento como un segmento independiente singleSegs */
    else
      singleSegs.push_back(seg);
  }
}

void LinePerceptor::getFirstAndLastOfLine(LinePercept::Line& line, Vector2<int>& first, Vector2<int>& last, bool updateLine)
{
	/* 
	Se calculan los parámetros que definen a la recta que aproxima a la linea formada en la etapa anterior (createLines).
	Además, se busca encontrar cuáles son los valores de los puntos extremos de la línea.
	*/

	/* Definición de punto de referencia. Se escoge el primer elemento del arreglo de lineSegments asociados a la Línea.
	Se definen variables auxiliares que permiten caracterizar la línea:
	first = primer punto de la linea
	last = último punto de la línea
	fist_dist = distancia del primer punto de la línea a la referencia
	last_dist = distancia del último punto de la línea a la referencia
	*/

  Vector2<int> p_ref = line.segments.at(0).p1;
  first = p_ref;
  last = p_ref;
  float first_dist = 0, last_dist = 0;

  Vector2<> mean;

  /* Se itera sobre la lista de lineSegments que conforman la línea encontrada */
  for(vector<LinePercept::LineSegment>::const_iterator seg = line.segments.begin(); seg != line.segments.end(); seg++)
  {
    /* Se define un valor promedio de x entre los puntos iniciales y finales del segmento. Se realiza lo mismo para y */
	mean.x += seg->p1.x + seg->p2.x;
    mean.y += seg->p1.y + seg->p2.y;

	/* Se calcula la distancia de los puntos iniciales y finales del segmento actual que se está analizando 
	respecto a la referencia definida anteriormente */
    const Vector2<int> diffp1 = seg->p1 - p_ref,
                       diffp2 = seg->p2 - p_ref;



	/* 
	A continuación se realiza un ordenamiento de los segmentos, con ayuda de las variables anteriores. Se basa principalmente en criterios 
	de cercanía de los puntos que describen al segmento actual, respecto al punto de referencia, al primer punto de la línea y 
	al último punto de la línea.
	
	
	Si dist(p1, p_ref) >dist(first, p_ref) y dist(first, p1) < dist(p1, pref)

	Significa que el punto p1 se encuentra más lejos de la referencia que el primer punto de la linea, y
	que se encuentra "al mismo lado" que el primer punto (respecto a la referencia).	

	En ese caso, se asigna el segmento actual como el primero, y se redefine la distancia del primer punto a la referencia,
	como la distancia del punto p1 a la referencia.
	*/
    if(diffp1.squareAbs() > first_dist && (seg->p1 - first).squareAbs() <= diffp1.squareAbs())
    {
      first = seg->p1;
      first_dist = (float) diffp1.squareAbs();
    }


	/* Si dist(p1, p_ref) > dist(last) y dist(p1, first) > dist(p_ref, first) y dist(p1, last) <= dist(p1, p_ref)

	Significa que el punto p1 se encuentra más lejos que el último punto de la línea,
	que se encuentra más lejos del primer punto de la línea que el punto de referencia, y
	que se encuentra más cerca del último punto que de lo que se encuentra de la referencia

	En éste caso, se reasigna el último punto de la línea como el punto p1, y se redefine la distancia
	del último punto a la referencia	como la distancia de p1 a la referencia.
	*/
    else if(diffp1.squareAbs() > last_dist && (seg->p1 - first).squareAbs() > (p_ref - first).squareAbs() && (seg->p1 - last).squareAbs() <= diffp1.squareAbs())
    {
      last = seg->p1;
      last_dist = (float) diffp1.squareAbs();
    }


	/* Si dist(p2, p_ref) > dist(first) y dist(p2, first) <= dist(p2, p_ref)

	Refiere a que si el punto p2 se encuentra a mayor distancia que el primer punto de la linea, 
	y se encuentra más cercano al primer punto que a la referencia.

	Se reasigna el primer punto al punto p2, y se redefine la distancia referencia-primer punto como 
	la distancia de p2 a la referencia.
	*/
    if(diffp2.squareAbs() > first_dist && (seg->p2 - first).squareAbs() <= diffp2.squareAbs())
    {
      first = seg->p2;
      first_dist = (float) diffp2.squareAbs();
    }


	/* Si dist(p2, p_ref) > dist(last) y dist(p2, first) > dist(p_ref, first) y dist(p2, last) <= dist(p2, p_ref)
	
	Si el punto p2 se encuentra más lejos de la referencia que el último punto,
	además se encuentra más lejos del primer punto que la referencia,
	y se encuentra más cerca del último punto que de la referencia.

	Se  reasigna el punto p2 como el último punto de la línea, y se redefina la distancia del último punto
	como la distancia de p2 a la referencia.

	*/
    else if(diffp2.squareAbs() > last_dist && (seg->p2 - first).squareAbs() > (p_ref - first).squareAbs() && (seg->p2 - last).squareAbs() <= diffp2.squareAbs())
    {
      last = seg->p2;
      last_dist = (float) diffp2.squareAbs();
    }
  }


  /* Actualización de los parámetros de la línea construida */
  if(updateLine)
  {
    ASSERT(line.segments.size());
	/* Si se tiene que la lista de segmentos es igual a 1, los parámetros alpha y d de la línea se asignan iguales a los del segmento */
    if((int) line.segments.size() == 1)
    {
      line.alpha = line.segments.at(0).alpha;
      line.d = line.segments.at(0).d;
    }
    else
    {
		/* 
		Si la lista de segmentos es mayor o igual a 1, se estima el alpha y d usando la recta que aproxima los puntos de inicio
		y término de cada segmento.
		*/
      //improve alpha and d estimation by calculating the fitting line through all start/end points of the segments

		/* Se define el valor promedio de los puntos de los segmentos que conforman la línea*/
      mean /= float((int)line.segments.size() * 2);
      //CROSS("module:LinePerceptor:Lines2", mean.x, mean.y, 80, 10, Drawings::ps_solid, ColorClasses::orange);
      float zaehlerSum = 0,		/* Contador */
            nennerSum = 0;		/* Denominador */

	  /* Se definen dos puntos auxiliares p1 y p2 para describir la recta */
      Vector2<> p1, p2;
      
	  /* Si el ángulo de la línea, menos pi/2 es mayor que pi/4 (revisar por qué) */
	  if(fabs(line.alpha - pi_2) > pi_4)
      {
		  /* Iterar sobre los segmentos que conforman la línea */
        for(vector<LinePercept::LineSegment>::const_iterator seg = line.segments.begin(); seg != line.segments.end(); seg++)
        {
          //CROSS("module:LinePerceptor:Lines2", seg->p1.x, seg->p1.y, 30, 10, Drawings::ps_solid, ColorClasses::robotBlue);
          //CROSS("module:LinePerceptor:Lines2", seg->p2.x, seg->p2.y, 30, 10, Drawings::ps_solid, ColorClasses::robotBlue);

		  /* Regresión lineal sobre los puntos que conforman la línea
		  zaehlerSum equivale al término sum((x-X)*(y-Y))
		  nennerSum equivale al término sum((x-X)^2))
		  */
          zaehlerSum += (seg->p1.y - mean.y) * (seg->p1.x - mean.x);
          nennerSum += sqr((seg->p1.y - mean.y));
          zaehlerSum += (seg->p2.y - mean.y) * (seg->p2.x - mean.x);
          nennerSum += sqr((seg->p2.y - mean.y));
        }
		/* Cálculo de coeficientes a y b de la recta */
        float b = zaehlerSum / nennerSum,
              a = mean.x - b * mean.y;

        ASSERT(a == a);
        ASSERT(b == b);

        p1.y = mean.y;
        p1.x = a + b * p1.y;
        p2.y = mean.y + 1000;
        p2.x = a + b * p2.y;
        //ARROW("module:LinePerceptor:Lines2", p1.x, p1.y, p2.x, p2.y, 5, Drawings::ps_solid, ColorClasses::blue);
      }

	  /* Si el ángulo de la línea, menos pi/2 es menor que pi/4 (también hay que revisar por qué) */
      else
      {
        for(vector<LinePercept::LineSegment>::const_iterator seg = line.segments.begin(); seg != line.segments.end(); seg++)
        {
          //CROSS("module:LinePerceptor:Lines2", seg->p1.x, seg->p1.y, 30, 10, Drawings::ps_solid, ColorClasses::robotBlue);
          //CROSS("module:LinePerceptor:Lines2", seg->p2.x, seg->p2.y, 30, 10, Drawings::ps_solid, ColorClasses::robotBlue);

		   /* Regresión lineal sobre los puntos que conforman la línea
		  zaehlerSum equivale al término sum((x-X)*(y-Y))
		  nennerSum equivale al término sum((x-X)^2))
		  */
          zaehlerSum += (seg->p1.x - mean.x) * (seg->p1.y - mean.y);
          nennerSum += sqr((seg->p1.x - mean.x));
          zaehlerSum += (seg->p2.x - mean.x) * (seg->p2.y - mean.y);
          nennerSum += sqr((seg->p2.x - mean.x));
        }
		/* Cálculo de coeficientes a y b de la recta */
        float b = zaehlerSum / nennerSum,
              a = mean.y - b * mean.x;

        ASSERT(a == a);
        ASSERT(b == b);

        p1.x = mean.x;
        p1.y = a + b * p1.x;
        p2.x = mean.x + 1000;
        p2.y = a + b * p2.x;

        //ARROW("module:LinePerceptor:Lines2", p1.x, p1.y, p2.x, p2.y, 5, Drawings::ps_solid, ColorClasses::yellow);
      }

	  /* Calculo de ángulo de la línea a partir de los puntos determinados anteriormente */
      line.alpha = (p1 - p2).angle() + pi_2;
	  /* Normalización del ángulo para que quede entre 0 y pi*/
      while(line.alpha < 0)
        line.alpha += pi;
      while(line.alpha >= pi)
        line.alpha -= pi;

      const float c = cos(line.alpha),
                  s = sin(line.alpha);

	  /* Cálculo del largo de la línea */
      line.d = p1.x * c + p1.y * s;
    }
  }

  /* Reasignación del primer y último punto de la línea */
  first = line.calculateClosestPointOnLine(first);
  last = line.calculateClosestPointOnLine(last);
}

void LinePerceptor::analyzeLines(list<LinePercept::Line>& lines, vector<LinePercept::Intersection>& intersections, LinePercept::CircleSpot& circle, list<LinePercept::LineSegment>& singleSegs,
                                 const FieldDimensions& theFieldDimensions)
{
  
  /*
  En ésta parte se analizan las líneas encontradas, de modo de conservar y caracterizar las que sirven realmente, como así también
  descartar algunas situaciones que no fueron detectadas en etapas previas. Algunos criterios que se aplican son:
  - Encontrar puntos de inicio y término
  - Borrar las líneas si se encuentran cercanas a un círculo
  - Borrar las líneas si tienen al menos dos segmentos que se sobreponen (podrían ser piernas del robot)
  - Encontrar líneas que son casi paralelas, ya que si sus puntos de inicio y término son cercanos, podrían haber líneas redundantes.
  - Juntar los segmentos solitarios que se encuentren próximos a otras líneas
  - Borrar las líneas que no se encuentran lo suficientemente cubiertas (razón (largo)/(suma de los segmentos) menor que una cota.
  - Encontrar intersecciones
  - Encontrar la línea central


  */
	
	/* Analizar todas las líneas para encontrar el punto inicial y final de cada una */

  //the points first and last are the two points on the line which
  //have the greatest distance to each other ("endpoints")
  for(list<LinePercept::Line>::iterator line = lines.begin(); line != lines.end(); line++)
    getFirstAndLastOfLine(*line, line->first, line->last);

  /* Primer criterio: Revisión de líneas próximas a círculos. Si es así, deben ser eliminadad. Sólo se realiza si se conoce 
  dónde se encuentra el círculo central.
  Falta revisar donde se generan las líneas asociadas al círculo central*/

  //delete lines if their circleSpot is near the found circle
  if(circle.found)
  {
	  /* Se define una lista de LinePercept para guardar las posibles líneas a borrar */
    vector<list<LinePercept::Line>::iterator> toDelete;
	/* Iteración sobre el conjunto de líneas encontradas */
    for(list<LinePercept::Line>::iterator l1 = lines.begin(); l1 != lines.end(); l1++)
    {
      /* Se define un vector que describe el punto medio de la línea */
		Vector2<int> line_mid = (l1->first + l1->last) / 2;
      //CROSS("module:LinePerceptor:CircleSpots", line_mid.x, line_mid.y, 30, 30, Drawings::ps_solid, ColorClasses::green);

	  /* Definición de un vector auxiliar perpendicular a la línea, y normalizado por el radio del círculo central de la cancha */
      Vector2<int> bla = Vector2<int>(line_mid + (l1->first - l1->last).rotate(pi_2).normalize(theFieldDimensions.centerCircleRadius + parameters.circleBiggerThanSpecified));
      //CROSS("module:LinePerceptor:CircleSpots", bla.x, bla.y, 30, 30, Drawings::ps_solid, ColorClasses::green);

	  /* Si la distancia al cuadrado entre el vector auxiliar y la posición del círculo central es menor que 
	  la máxima distancia al círculo definida en maxLineCircleDist, agregar la línea a la lista de líneas a borrar */
      if((bla - circle.pos).squareAbs() < sqr(parameters.maxLineCircleDist))
        toDelete.push_back(l1);
      
	  /* Sino, definir un vector auxiliar también perpendicular, pero orientado en el sentido contrario. */
	  else
      {
        Vector2<int> bla = Vector2<int>(line_mid + (l1->first - l1->last).rotate(-pi_2).normalize(theFieldDimensions.centerCircleRadius + parameters.circleBiggerThanSpecified));
        //CROSS("module:LinePerceptor:CircleSpots", bla.x, bla.y, 30, 30, Drawings::ps_solid, ColorClasses::green);
	  /* Y se utiliza el mismo criterio: 
	  Si la distancia al cuadrado entre el vector auxiliar y la posición del círculo central es menor que 
	  la máxima distancia al círculo definida en maxLineCircleDist, agregar la línea a la lista de líneas a borrar */
        if((bla - circle.pos).squareAbs() < sqr(parameters.maxLineCircleDist))
          toDelete.push_back(l1);
      }

    }
	/* Finalmente, se borran las líneas del arreglo de líneas, que fueron indicadas en la lista toDelete */
    for(vector<list<LinePercept::Line>::iterator>::iterator t = toDelete.begin(); t != toDelete.end(); t++)
      lines.erase(*t);
  }

  /* Segundo criterio: Buscar las líneas que tengan segmentos que se sobreponen. También deben eliminarse, 
  porque podrían estar incluidas piernas de robots
  
  
  Observación: El algoritmo no está revisando en profundidad
  */

  //delete lines if they have at least two segments which overlap (these might be robot legs)

  /* Nuevamente se define una lista auxiliar de líneas a borrar  y se comienza a iterar*/
  vector<list<LinePercept::Line>::iterator> toDelete;
  for(list<LinePercept::Line>::iterator line = lines.begin(); line != lines.end(); line++)
  {
	  /* Si la línea tiene el parámetro dead=TRUE, se pasa a la siguiente */
    if(line->dead)
      continue;

	/* Se define un ángulo auxiliar ortogonal al ángulo de la línea */
    float alpha2 = line->alpha + pi_2;

	/* Revisión sobre los segmentos que conforman la línea, para chequear superposiciones entre segmentos*/
    vector<LinePercept::LineSegment>::iterator other;
    for(vector<LinePercept::LineSegment>::iterator seg = line->segments.begin(); seg != line->segments.end(); seg++)
    {
				
      float d1 = seg->p1.x * cos(alpha2) + seg->p1.y * sin(alpha2),
            d2 = seg->p2.x * cos(alpha2) + seg->p2.y * sin(alpha2);

	  
      Range<> clipper(min(d1, d2), max(d1, d2));

      other = seg;
      other++;

	  /* Iteración sobre el conjunto de los otros segmentos */
      for(; other != line->segments.end(); other++)
      {
	
        float otherd1 = clipper.limit(other->p1.x * cos(alpha2) + other->p1.y * sin(alpha2)),
              otherd2 = clipper.limit(other->p2.x * cos(alpha2) + other->p2.y * sin(alpha2));

        if(fabs(otherd1 - otherd2) > parameters.maxOverlapLength)
        {
          //LINE("module:LinePerceptor:Lines3", seg->p1.x, seg->p1.y, seg->p2.x, seg->p2.y, 5, Drawings::ps_solid, ColorClasses::red);
          //LINE("module:LinePerceptor:Lines3", other->p1.x, other->p1.y, other->p2.x, other->p2.y, 5, Drawings::ps_solid, ColorClasses::red);
          toDelete.push_back(line);
          goto breakOuter;
        }
      }
    }
breakOuter:
    ;
  }
  /* Se borran las líneas del arreglo de líneas, que fueron indicadas en la lista toDelete */
  for(vector<list<LinePercept::Line>::iterator>::iterator t = toDelete.begin(); t != toDelete.end(); t++)
    lines.erase(*t);


  /* Tercer criterio: Búsqueda de líneas paralelas y de puntos de inicio y término de la línea, para evitar
  la redundancia de líneas.

  Si se encuentra una línea paralela a otra, se redefine una de las líneas agregando los segmentos que le faltan,
  se busca el nuevo ángulo, distancia y puntos extremos.

  La línea que fue utilizada para completar la otra, se redefine como "delete"

  */

  //find lines which are allmost parallel
  toDelete.clear();

  /* Iteración en el arreglo de líneas */
  for(list<LinePercept::Line>::iterator line = lines.begin(); line != lines.end(); line++)
  {
	  /* Si la línea tiene el parámetro dead=TRUE, saltarla */
    if(line->dead)
      continue;
    list<LinePercept::Line>::iterator other = line;
    other++;

	/* Revisar las otras líneas */
    for(; other != lines.end(); other++)
    {
		/* Si la otra línea revisada tiene el parámetro dead=TRUE, saltarla */
      if(other->dead)
        continue;

	  /* Diferencia angular entre las direcciones de las líneas */
      float alphaDiff = line->alpha - other->alpha;

	  /* Normalización de la diferencia angular */
      while(alphaDiff < - pi_2)
        alphaDiff += pi;
      while(alphaDiff >= pi_2)
        alphaDiff -= pi;

	  /* Si la línea se aproxima a los extremos de la otra línea, o viceversa (la distancia es menor que la máxima distancia de separación 
	  entre líneas (maxLineUniteDist), y además la diferencia angular (alphaDiff) es menor que el máximo establecido (maxLineUniteAlphaDiff)*/

      //if endpoints of the other line are close to the line
      if(((abs(line->calculateDistToLine(other->first)) < parameters.maxLineUniteDist &&
           abs(line->calculateDistToLine(other->last)) < parameters.maxLineUniteDist)
          ||
          (abs(other->calculateDistToLine(line->first)) < parameters.maxLineUniteDist &&
           abs(other->calculateDistToLine(line->last)) < parameters.maxLineUniteDist)
         )
         &&
         abs(alphaDiff) < parameters.maxLineUniteAlphaDiff
        )
      {
		  /* Insertar los elementos de la otra línea en la línea actual */
        line->segments.insert(line->segments.end(), other->segments.begin(), other->segments.end());

		/* Se redefine la distancia de la línea como el promedio de las 2 */
        line->d = (line->d + other->d) / 2;

		/* Si la diferencia de ángulos entre la línea actual y la otra línea es mayor a pi/2, normalizar */
        if(line->alpha - other->alpha > pi_2)
          other->alpha += pi;
        else if(other->alpha - line->alpha > pi_2)
          other->alpha -= pi;

		/* Se reasigna el ángulo de la línea como el promedio de los ángulos */
        line->alpha = (line->alpha + other->alpha) / 2;

		/* Si el ángulo de la línea es mayor que cero, ajustar sumando pi */
        if(line->alpha < 0)
          line->alpha += pi;

		/* Obtener los extremos de la línea */
        getFirstAndLastOfLine(*line, (*line).first, (*line).last);

		/* Agregar la otra línea a la lista de líneas a borrar toDelete */
        toDelete.push_back(other);

		/* Redefinir el parámetro dead de la otra línea como TRUE */
        other->dead = true;

      }
    }
  }
	/* Limpieza de las líneas en el arreglo toDelete */
  for(vector<list<LinePercept::Line>::iterator>::iterator t = toDelete.begin(); t != toDelete.end(); t++)
    lines.erase(*t);


  /* Cuarto criterio: Búsqueda de singleSegments próximos a alguna de las líneas
  para integrarlo a una de las líneas, o bien eliminarlo como un pedazo de línea válida
  */

  //add singleSegments where the start and end pos is close to a line to the line
  vector<list<LinePercept::LineSegment>::iterator> ttoDelete;

  /* Iteración sobre todos los singleSegments */
  for(list<LinePercept::LineSegment>::iterator seg = singleSegs.begin(); seg != singleSegs.end(); seg++)
  {
	  /* Iteración sobre todas las líneas */
    for(list<LinePercept::Line>::iterator line = lines.begin(); line != lines.end(); line++)
    {
		/* Si la distancia de los puntos extremos del segmento a la línea de la iteración actual, es menor a una cierta cota */
      if(abs(seg->p1.x * cos(line->alpha) + seg->p1.y * sin(line->alpha) - line->d) < parameters.maxLineSingleSegDist &&
         abs(seg->p2.x * cos(line->alpha) + seg->p2.y * sin(line->alpha) - line->d) < parameters.maxLineSingleSegDist)
      {
		  /* Se calcula la distancia del primer al último punto de la línea (distancia euclideana) */
        float firstToLast = (float)(line->last - line->first).abs();

		/* Se calcula el valor del punto medio del segmento */
        const Vector2<int> segMid = (seg->p1 + seg->p2) / 2;
        //CROSS("module:LinePerceptor:Lines2", segMid.x, segMid.y, 30, 20, Drawings::ps_solid, ColorClasses::yellow);

		/* Si la longitud de la línea es menor que la distancia de la mitad del segmento a uno de los extremos */
        if((firstToLast < (line->last - segMid).abs() || firstToLast < (line->first - segMid).abs()))
        {
			/* Significa que el segmento no está entre los extremos de la línea */
          //seg is not between first and last

			/* Calculamos entonces la distancia mínima que existe entre el segmento y la línea*/
          const float minToLine = (line->last - segMid).abs() > (line->first - segMid).abs() ? (float)(line->first - segMid).abs() : (float)(line->last - segMid).abs();

          /*
          COMPLEX_DRAWING("module:LinePerceptor:Lines2",
          {
            DRAWTEXT("module:LinePerceptor:Lines2", segMid.x + 20, segMid.y, 10, ColorClasses::black, minToLine);
          });
          */

		  /* Si la distancia mínima es mayor que la máxima distancia entre el segmento y la línea para ser añadida, se salta a la siguiente línea*/
          if(minToLine > parameters.maxLineSingleSegDist2)
            continue;
          //CROSS("module:LinePerceptor:Lines2", segMid.x, segMid.y, 30, 20, Drawings::ps_solid, ColorClasses::red);
        }
        else
          //CROSS("module:LinePerceptor:Lines2", segMid.x, segMid.y, 30, 20, Drawings::ps_solid, ColorClasses::blue);

		/* Si pasa las pruebas anteriores, se agrega el puntero del segmento a la línea */
        line->segments.push_back(*seg);

		/* Se agrega el segmento a la lista de segmentos a borrar */
        ttoDelete.push_back(seg);

		/* Se obtienen nuevamente los puntos extremos de la línea */
        getFirstAndLastOfLine(*line, line->first, line->last, false);
        break;
      }
    }
  }
  /* Se borran los segmentos que quedaron en la lista de segmentos a borrar */
  for(vector<list<LinePercept::LineSegment>::iterator>::iterator d = ttoDelete.begin(); d != ttoDelete.end(); d++)
    singleSegs.erase(*d);

  /* Quinto Criterio: Eliminar las líneas que no están bien cubiertas
  
  */
  //delete lines which do not "hard cover" (length / sum(segments.length)) enough
  toDelete.clear();
  /* Iteración sobre el arreglo de líneas */
  for(list<LinePercept::Line>::iterator l1 = lines.begin(); l1 != lines.end(); l1++)
  {
	  /* Iteración sobre el arreglo de segmentos de la línea actual */
    int hardcover = 0;
    for(vector<LinePercept::LineSegment>::iterator seg = l1->segments.begin(); seg != l1->segments.end(); seg++)
		/* Se acumula la suma de los largos de cada segmento */
      hardcover += (seg->p1 - seg->p2).abs();
	/* Se define una razón hardCover como la suma de los largos de los segmentos sobre el largo de la línea */
    const float hardcoverRatio = hardcover / (float)(l1->first - l1->last).abs();

	/* Si la razón es menor al mínimo permitido, agregar la línea al arreglo de íneas a eliminar */
    if(hardcoverRatio < parameters.minHardcover)
      toDelete.push_back(l1);
  }

  /* Iteración sobre el arreglo toDelete*/
  for(vector<list<LinePercept::Line>::iterator>::iterator t = toDelete.begin(); t != toDelete.end(); t++)
  {
	  /* Iteración sobre todos los segmentos de la línea actual */
    for(vector<LinePercept::LineSegment>::iterator seg = (*(*t)).segments.begin(); seg != (*(*t)).segments.end(); seg++)
      /* Se agrega el puntero del segmento al arreglo singleSegs */
		singleSegs.push_back(*seg);
	/* Se borra la línea */
    lines.erase(*t);
  }

  /* Sexto Criterio: Encontrar intersecciones entre las líneas para conformar las T y las X

  */
  //find intersections

  /* Iteración sobre las líneas */
  for(list<LinePercept::Line>::iterator l1 = lines.begin(); l1 != lines.end(); l1++)
  {
	  /* Asignación de la línea de la iteración actual, a una línea   */
    list<LinePercept::Line>::iterator l2 = l1;

	/* Iteracións sobre las líneas siguientes a la actual */
    for(l2++; l2 != lines.end(); l2++)
    {
		/* Se define un ángulo que corresponde al ángulo que forman las dos líneas. Si el ángulo es menor que -pi/2 se normaliza
		hasta llegar a [-pi/2, 0]. Si es mayor a pi/2, se normaliza al intervalo [0, pi/2] */
      float alphaDiff = l1->alpha - l2->alpha;
      while(alphaDiff < -pi_2)
        alphaDiff += pi;
      while(alphaDiff >= pi_2)
        alphaDiff -= pi;
	  /* Si el ángulo es menor al ángulo mínimo definido, para considerar que las rectas de intersectan y no son paralelas,
	  se pasa a la siguiente línea */
      if(abs(alphaDiff) < parameters.minIntersectionAlphaDiff)
        continue;

	  /* Si la distancia de la línea l1 y la línea l2 son menores que una cierta longitud mínima de intersección, se salta a la otra línea */
      if((l1->first - l1->last).squareAbs() < sqr(parameters.minIntersectionLength) &&
         (l2->first - l2->last).squareAbs() < sqr(parameters.minIntersectionLength))
        continue;

	  /* Cálculo de las coordenadas de la intersección de las dos rectas */
      //zwei hessesche normaleformen gleichsetzen und aufloesen, dann kommt das bei raus
      const float zaehler = l1->d - (l2->d * cos(l1->alpha) / cos(l2->alpha)),
                  nenner = sin(l1->alpha) - (sin(l2->alpha) * cos(l1->alpha) / cos(l2->alpha));
      const float y_s = zaehler / nenner,
                  x_s = (l1->d - y_s * sin(l1->alpha)) / cos(l1->alpha);

	  /* Si existe la intersección, no son paralelas o son idénticas */
      if(y_s == y_s && x_s == x_s)
      {
        const Vector2<int> s_p = Vector2<int>((int)x_s, (int)y_s);
        //this is some freay stuff which determines in which relation the
        //point s_p is to l1->first/last and l2->first/last given s_p is the
        //intersectionpoint of l1 and l2
        //distToLx = ( -min(dist(sp,last/first)) if in between,  )
        //           (  min(dist(s_p,last/first) else)           )
		

		/* Se busca encontrar la distancia que existe entre la intersección y los puntos extremos de la línea l1
		*/
        float spToFirst = (float)(s_p - l1->first).abs(),
              spToLast = (float)(s_p - l1->last).abs(),
              firstToLast = (float)(l1->first - l1->last).abs();
        float distToL1 = 0, distToL2 = 0;
        if(spToFirst < firstToLast && spToLast < firstToLast)
          //sp is between first and last
          distToL1 = - (spToFirst > spToLast ? spToLast : spToFirst);
        else if(spToFirst >= firstToLast)
          //sp is closer to last
          distToL1 = spToLast;
        else if(spToLast >= firstToLast)
          //sp is closer to first
          distToL1 = spToFirst;
        else
          ASSERT(false);
        spToFirst = (float)(s_p - l2->first).abs(),
        spToLast = (float)(s_p - l2->last).abs(),
        firstToLast = (float)(l2->first - l2->last).abs();
        if(spToFirst < firstToLast && spToLast < firstToLast)
          //sp is between first and last
          distToL2 = - (spToFirst > spToLast ? spToLast : spToFirst);
        else if(spToFirst >= firstToLast)
          //sp is closer to last
          distToL2 = spToLast;
        else if(spToLast >= firstToLast)
          //sp is closer to first
          distToL2 = spToFirst;
        else
          ASSERT(false);
        //end freaky stuff

        LinePercept::Intersection inter;
        inter.pos = Vector2<int>((int)x_s, (int)y_s);
        Vector2<int> t1 = l1->first - l1->last,
                     t2 = l2->first - l2->last;
        //this checks whether the intersection point is closer to first
        //or to last and if it is closer to first we need to flip the
        //direction
        if((l1->first - inter.pos).squareAbs() < (l1->last - inter.pos).squareAbs())
          t1 = l1->last - l1->first;
        if((l2->first - inter.pos).squareAbs() < (l2->last - inter.pos).squareAbs())
          t2 = l2->last - l2->first;
        //this is the heading of the intersection (to l1 and l2)
        Vector2<> dirL1 = Vector2<>((float) t1.x, (float) t1.y).normalize(),
                  dirL2 = Vector2<>((float) t2.x, (float) t2.y).normalize();


        if(distToL1 < -parameters.minTToEnd && distToL2 < -parameters.minTToEnd)
        {
          //ARROW("module:LinePerceptor:Intersections", x_s, y_s, l1->last.x, l1->last.y, 5, Drawings::ps_solid, ColorClasses::yellow);
          //ARROW("module:LinePerceptor:Intersections", x_s, y_s, l2->last.x, l2->last.y, 5, Drawings::ps_solid, ColorClasses::yellow);
          //this is a X
          inter.type = LinePercept::Intersection::X;
          inter.dir1 = dirL1;
          inter.dir2 = dirL2;
          intersections.push_back(inter);
        }
        else if((distToL1 < -parameters.minTToEnd && distToL2 < parameters.maxTFromEnd) ||
                (distToL2 < -parameters.minTToEnd && distToL1 < parameters.maxTFromEnd))
        {
          //ARROW("module:LinePerceptor:Intersections", x_s, y_s, l1->last.x, l1->last.y, 5, Drawings::ps_solid, ColorClasses::yellow);
          //ARROW("module:LinePerceptor:Intersections", x_s, y_s, l2->last.x, l2->last.y, 5, Drawings::ps_solid, ColorClasses::yellow);
          //this is a T
          inter.type = LinePercept::Intersection::T;
          if(distToL2 < -parameters.minTToEnd && distToL1 < parameters.maxTFromEnd)
          {
            //l2 is the intersected line (the upper part of the T)
            inter.dir1 = dirL1;
            inter.dir2 = dirL2;
          }
          else
          {
            //l1 is the intersected line (the upper part of the T)
            inter.dir1 = dirL2;
            inter.dir2 = dirL1;
          }
          intersections.push_back(inter);
        }
        else if(distToL1 < parameters.maxTFromEnd && distToL2 < parameters.maxTFromEnd)
        {
          //ARROW("module:LinePerceptor:Intersections", x_s, y_s, l1->last.x, l1->last.y, 5, Drawings::ps_solid, ColorClasses::yellow);
          //ARROW("module:LinePerceptor:Intersections", x_s, y_s, l2->last.x, l2->last.y, 5, Drawings::ps_solid, ColorClasses::yellow);
          //this is a L
          inter.type = LinePercept::Intersection::L;
          inter.dir1 = dirL1;
          inter.dir2 = dirL2;
          intersections.push_back(inter);
        }
      }
    }
  }

  //find "mittellinie"
  if(circle.found)
  {
    list<LinePercept::Line>::iterator closestLine;
    int minDist = -1;
    for(list<LinePercept::Line>::iterator l1 = lines.begin(); l1 != lines.end(); l1++)
    {
      const int dist = (int)abs(l1->calculateDistToLine(circle.pos));
      if(dist < parameters.maxMidLineToCircleDist &&
         (dist < minDist || minDist == -1) &&
         (l1->first - l1->last).squareAbs() > sqr(parameters.minMidLineLength))
      {
        closestLine = l1;
        minDist = dist;
      }
    }

    if(minDist != -1)
    {
      closestLine->midLine = true;
      circle.pos = closestLine->calculateClosestPointOnLine(circle.pos);

      //intersections
      const Vector2<int> midLineDir = (closestLine->first - closestLine->last).normalize(theFieldDimensions.centerCircleRadius + parameters.circleBiggerThanSpecified);
      LinePercept::Intersection inter;

      inter.pos = circle.pos + midLineDir;
      inter.dir1 = Vector2<>((float) midLineDir.x, (float) midLineDir.y).normalize();
      inter.dir2 = inter.dir1;
      inter.dir2.rotateLeft();
      inter.type = LinePercept::Intersection::X;
      intersections.push_back(inter);

      inter.pos = circle.pos - midLineDir;
      inter.dir1 = Vector2<>((float) midLineDir.x, (float) midLineDir.y).normalize();
      inter.dir2 = inter.dir1;
      inter.dir2.rotateLeft();
      inter.type = LinePercept::Intersection::X;
      intersections.push_back(inter);
    }
  }
}

void LinePerceptor::analyzeSingleSegments(list<LinePercept::LineSegment>& singleSegs, LinePercept::CircleSpot& circle, list<LinePercept::Line>& lines,
                                          const FieldDimensions& theFieldDimensions, const FrameInfo& theFrameInfo)
{

  list<LinePercept::CircleSpot> circleSpots;
  list<LinePercept::CircleSpot> circleSpots2;
  LinePercept::CircleSpot spot;

  list<LinePercept::LineSegment>::iterator seg2;
  for(list<LinePercept::LineSegment>::iterator seg = singleSegs.begin(); seg != singleSegs.end(); seg++)
  {
    //ARROW("module:LinePerceptor:CircleSpots", seg->p1.x, seg->p1.y, seg->p2.x, seg->p2.y, 20, Drawings::ps_solid, ColorClasses::robotBlue);

    const Vector2<int> seg_dir = seg->p1 - seg->p2;
    const Vector2<int> seg_mid = (seg->p1 + seg->p2) / 2;
    const Vector2<int> seg_norm = Vector2<int>(seg_dir.x, seg_dir.y).rotateLeft();

    ASSERT(seg->p1 != seg->p2);
    const Vector2<int> spot1 = seg_mid + (seg->p1 - seg->p2).rotate(pi_2).normalize(theFieldDimensions.centerCircleRadius + parameters.circleBiggerThanSpecified);
    spot.pos = spot1;
    spot.iterator = seg;
    //LINE("module:LinePerceptor:CircleSpots2", spot.pos.x, spot.pos.y, seg_mid.x, seg_mid.y, 5, Drawings::ps_solid, ColorClasses::yellow);
    //CROSS("module:LinePerceptor:CircleSpots2", spot.pos.x, spot.pos.y, 20, 20, Drawings::ps_solid, ColorClasses::yellow);
    circleSpots2.push_back(spot);
    const Vector2<int> spot2 = seg_mid + (seg->p1 - seg->p2).rotate(-pi_2).normalize(theFieldDimensions.centerCircleRadius + parameters.circleBiggerThanSpecified);
    spot.pos = spot2;
    spot.iterator = seg;
    //LINE("module:LinePerceptor:CircleSpots2", spot.pos.x, spot.pos.y, seg_mid.x, seg_mid.y, 5, Drawings::ps_solid, ColorClasses::yellow);
    //CROSS("module:LinePerceptor:CircleSpots2", spot.pos.x, spot.pos.y, 20, 20, Drawings::ps_solid, ColorClasses::yellow);
    circleSpots2.push_back(spot);

    if(seg_dir.squareAbs() < sqr(circleParams.minSegmentLength) || (seg->p1Img - seg->p2Img).squareAbs() < sqr(circleParams.minSegmentImgLength))
      continue;

    //LINE("module:LinePerceptor:CircleSpots", seg_mid.x, seg_mid.y, seg_mid.x + seg_norm.x, seg_mid.y + seg_norm.y, 20, Drawings::ps_solid, ColorClasses::orange);

    seg2 = seg;
    seg2++;
    for(; seg2 != singleSegs.end(); seg2++)
    {
      const Vector2<int> seg2_dir = seg2->p1 - seg2->p2;
      if(seg2_dir.squareAbs() < sqr(circleParams.minSegmentLength) || (seg2->p1Img - seg2->p2Img).squareAbs() < sqr(circleParams.minSegmentImgLength))
        continue;

      if((seg->p1 - seg2->p1).squareAbs() < sqr(circleParams.maxNgbhDist) ||
         (seg->p1 - seg2->p2).squareAbs() < sqr(circleParams.maxNgbhDist) ||
         (seg->p2 - seg2->p1).squareAbs() < sqr(circleParams.maxNgbhDist) ||
         (seg->p2 - seg2->p2).squareAbs() < sqr(circleParams.maxNgbhDist))
      {
        const Vector2<int> seg2_mid = (seg2->p1 + seg2->p2) / 2;
        //LINE("module:LinePerceptor:CircleSpots", seg_mid.x, seg_mid.y, seg2_mid.x, seg2_mid.y, 20, Drawings::ps_solid, ColorClasses::red);
        const Vector2<int> seg2_norm = Vector2<int>(seg2_dir.x, seg2_dir.y).rotateLeft();

        const Vector2<int> p1 = seg_mid;
        const Vector2<int> p2 = seg_mid + seg_norm;;
        const Vector2<int> p3 = seg2_mid;
        const Vector2<int> p4 = seg2_mid + seg2_norm;

        const int zaehler1 = (p4.x - p3.x) * (p1.y - p3.y) - (p4.y - p3.y) * (p1.x - p3.x);
        const int nenner1 = (p4.y - p3.y) * (p2.x - p1.x) - (p4.x - p3.x) * (p2.y - p1.y);
        float X1factor = zaehler1 / (float)nenner1;

        const int zaehler2 = (p2.x - p1.x) * (p1.y - p3.y) - (p2.y - p1.y) * (p1.x - p3.x);
        const int nenner2 = (p4.y - p3.y) * (p2.x - p1.x) - (p4.x - p3.x) * (p2.y - p1.y);
        const float X2factor = zaehler2 / (float)nenner2;

        const Vector2<int> t = p2 - p1;
        const Vector2<int> inter = p1 + Vector2<int>((int)(t.x * X1factor), (int)(t.y * X1factor));

        if(abs(abs(seg_norm.abs() * X1factor) - theFieldDimensions.centerCircleRadius + parameters.circleBiggerThanSpecified) > circleParams.maxRadiusError || abs(abs(seg2_norm.abs() * X2factor) - theFieldDimensions.centerCircleRadius + parameters.circleBiggerThanSpecified) > circleParams.maxRadiusError)
          continue;

        const int X1Sign = X1factor > 0 ? 1 : -1;
        const int X2Sign = X2factor > 0 ? 1 : -1;
        const Vector2<int> i1 = seg_mid + Vector2<int>(seg_norm).normalize((theFieldDimensions.centerCircleRadius + parameters.circleBiggerThanSpecified) * X1Sign);
        const Vector2<int> i2 = seg2_mid + Vector2<int>(seg2_norm).normalize((theFieldDimensions.centerCircleRadius + parameters.circleBiggerThanSpecified) * X2Sign);

        //CROSS("module:LinePerceptor:CircleSpots", inter.x, inter.y, 40, 20, Drawings::ps_solid, ColorClasses::red);
        //CROSS("module:LinePerceptor:CircleSpots", i1.x, i1.y, 40, 20, Drawings::ps_solid, ColorClasses::blue);
        //CROSS("module:LinePerceptor:CircleSpots", i2.x, i2.y, 40, 20, Drawings::ps_solid, ColorClasses::blue);
        //LINE("module:LinePerceptor:CircleSpots", seg_mid.x, seg_mid.y, inter.x, inter.y, 10, Drawings::ps_solid, ColorClasses::orange);
        //LINE("module:LinePerceptor:CircleSpots", seg2_mid.x, seg2_mid.y, inter.x, inter.y, 10, Drawings::ps_solid, ColorClasses::orange);
        spot.pos = i1;
        circleSpots.push_back(spot);
        spot.pos = i2;
        circleSpots.push_back(spot);
      }
    }
  }

  //Hough Transformation fuer (ganz) arme ;-)
  const int sqrMaxSupporterDist = sqr(circleParams.maxSupporterDist);
  std::list<list<LinePercept::LineSegment>::iterator> toDelete;
  for(list<LinePercept::CircleSpot>::iterator spot_iter = circleSpots.begin(); spot_iter != circleSpots.end(); spot_iter++)
  {
    spot = *spot_iter;
    Vector2<int> center(0, 0);

    vector<list<LinePercept::CircleSpot>::iterator> supporters;

    for(list<LinePercept::CircleSpot>::iterator other = circleSpots.begin(); other != circleSpots.end(); other++)
    {
      if((other->pos - spot.pos).squareAbs() < sqrMaxSupporterDist)
      {
        supporters.push_back(other);
        center += other->pos;
      }
    }

    if((int)supporters.size() >= circleParams.minSupporters)
    {
      center /= supporters.size();

      //collect second round of supporters
      for(list<LinePercept::CircleSpot>::iterator other = circleSpots2.begin(); other != circleSpots2.end(); other++)
        if((other->pos - center).squareAbs() < sqr(circleParams.maxSupporterDist2))
          toDelete.push_back(other->iterator);

      circle.pos = center;
      circle.found = true;
      circle.lastSeen = theFrameInfo.time;
      //CIRCLE("module:LinePerceptor:CircleSpots", circle.pos.x, circle.pos.y, circleParams.maxSupporterDist, 30, Drawings::ps_solid, ColorClasses::blue, Drawings::bs_null, ColorClasses::robotBlue);
      //CIRCLE("module:LinePerceptor:CircleSpots", circle.pos.x, circle.pos.y, circleParams.maxSupporterDist2, 30, Drawings::ps_solid, ColorClasses::blue, Drawings::bs_null, ColorClasses::robotBlue);
      break;
    }
  }
  for(list<list<LinePercept::LineSegment>::iterator>::iterator d = toDelete.begin(); d != toDelete.end(); ++d)
  {
    list<list<LinePercept::LineSegment>::iterator>::iterator i = d;
    for(++i; i != toDelete.end();)
      if(*i == *d)
        i = toDelete.erase(i);
      else
        ++i;
    singleSegs.erase(*d);
  }

  //a single segment is assumed to be a line if it's size is sufficent (and it's not part of the circle)
  toDelete.clear();
  for(list<LinePercept::LineSegment>::iterator seg = singleSegs.begin(); seg != singleSegs.end(); seg++)
  {
    if((seg->p1 - seg->p2).squareAbs() > sqr(parameters.minLineSingleRegionLength))
    {
      LinePercept::Line l;
      l.d = seg->d;
      l.alpha = seg->alpha;
      l.segments.push_back(*seg);
      l.dead = false;
      l.midLine = false;
      lines.push_back(l);
      toDelete.push_back(seg);
    }
  }
  for(list<list<LinePercept::LineSegment>::iterator>::iterator d = toDelete.begin(); d != toDelete.end(); ++d)
  {
    list<list<LinePercept::LineSegment>::iterator>::iterator i = d;
    for(++i; i != toDelete.end();)
      if(*i == *d)
        i = toDelete.erase(i);
      else
        ++i;
    singleSegs.erase(*d);
  }
}

//MAKE_MODULE(LinePerceptor, Perception)

