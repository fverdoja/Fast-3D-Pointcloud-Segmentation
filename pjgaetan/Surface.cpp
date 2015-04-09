//#include "stdafx.h" //Visual Studio specific
#include "Surface.h"


#define lowInfo 1.0


/***************************************************************************************/
/*************************** Annex Functions Used only in surface.cpp **************************/
/***************************************************************************************/

void cross(double res[3], double A[3], double B[3]) {
	res[0] = A[1]*B[2] - A[2]*B[1];
	res[1] = -A[0]*B[2] + A[2]*B[0];
	res[2] = A[0]*B[1] - A[1]*B[0];
}

void cross_3d(double res[3], double A[3], double B[3]) {
	res[0] = A[1]*B[2] - A[2]*B[1];
	res[1] = -A[0]*B[2] + A[2]*B[0];
	res[2] = A[0]*B[1] - A[1]*B[0];
}

double cross_2d(double res, double A[2], double B[2]) {

	res = A[0]*B[1] - A[1]*B[0];
	return res;
}

double *diff(const double* A, const double*  B) {
	//double *res = new double[3];
	double res[3];
	res[0] = A[0]-B[0];
	res[1] = A[1]-B[1];
	res[2] = A[2]-B[2];
	return res;
}

double *diff(int A[2], int B[2]) {
	//double *res = new double[2];
	double res[2];
	res[0] = double(A[0]-B[0]);
	res[1] = double(A[1]-B[1]);
	return res;
}

//double dist_3d(double A[3], double B[3]){
double dist_3d(const double * A, const double * B){
	double val = sqrt( (A[0]-B[0])*(A[0]-B[0]) + (A[1]-B[1])*(A[1]-B[1]) + (A[2]-B[2])*(A[2]-B[2]) );
	return val;
}

double dist_2d(const double * A, const double *  B){
	double val = sqrt( (A[0]-B[0])*(A[0]-B[0]) + (A[1]-B[1])*(A[1]-B[1]) );
	return val;
}


void normalize(double A[3]) {
	double val = sqrt(A[0]*A[0] + A[1]*A[1] + A[2]*A[2]);
	if (val > 0.0) { 
		A[0] = A[0]/val;
		A[1] = A[1]/val;
		A[2] = A[2]/val;
	}
}


void normalize_2(double A[2]) {
	double val = sqrt(A[0]*A[0] + A[1]*A[1]);
	if (val > 0.0) {
		A[0] = A[0]/val;
		A[1] = A[1]/val;
	}
}

double prod_scal(const double* A, const double* B) {//
	return A[0]*B[0] + A[1]*B[1] + A[2]*B[2];
}


double prod_scal(double A[3], double B[3]) {
	return A[0]*B[0] + A[1]*B[1] + A[2]*B[2];
}

double prod_scal_2(double A[2], double B[2]) {
	return A[0]*B[0] + A[1]*B[1];
}

void base_3d(const double * V1, const double * V2, const double * V3, double V2toB[3]) {

	double x1 = V1[0], x2 = V2[0] , x3 = V3[0] ;
	double y1 = V1[1], y2 = V2[1] , y3 = V3[1] ;
	double z1 = V1[2], z2 = V2[2] , z3 = V3[2] ;

	V2toB[0] = ( (x2-x1)*(x3-x1)+(y2-y1)*(y3-y1)+(z2-z1)*(z3-z1) )*(x3-x1) / ( (x3-x1)*(x3-x1)+(y3-y1)*(y3-y1)+(z3-z1)*(z3-z1) )+x1-x2 ;
	V2toB[1] = ( (x2-x1)*(x3-x1)+(y2-y1)*(y3-y1)+(z2-z1)*(z3-z1) )*(y3-y1) / ( (x3-x1)*(x3-x1)+(y3-y1)*(y3-y1)+(z3-z1)*(z3-z1) )+y1-y2 ;
	V2toB[2] = ( (x2-x1)*(x3-x1)+(y2-y1)*(y3-y1)+(z2-z1)*(z3-z1) )*(z3-z1) / ( (x3-x1)*(x3-x1)+(y3-y1)*(y3-y1)+(z3-z1)*(z3-z1) )+z1-z2 ;
	
}


void base_2d(const double * V1,  const double * V2, const double * V3, double V2toB[2]) {

	double x1 = V1[0], x2 = V2[0] , x3 = V3[0] ;
	double y1 = V1[1], y2 = V2[1] , y3 = V3[1] ;

	V2toB[0] = ((x2-x1)*(x3-x1)+(y2-y1)*(y3-y1))*(x3-x1)/((x3-x1)*(x3-x1)+(y3-y1)*(y3-y1))+x1-x2 ;
	V2toB[1] = ((x2-x1)*(x3-x1)+(y2-y1)*(y3-y1))*(y3-y1)/((x3-x1)*(x3-x1)+(y3-y1)*(y3-y1))+y1-y2 ;

}


void writeInt2DTabToFile(int** m, int row, int cols ,const char* filename)
{
    ofstream fout(filename);

    if(!fout)
    {
        cout<<"File Not Opened"<<endl;  return;
    }

    for(int i=0; i<row; i++)
    {
        for(int j=0; j<cols; j++)
        {
            fout<<m[i][j]<<"\t";
        }
        fout<<endl;
    }

    fout.close();
}



/***************************************************************************************/
/*************************** Methods for the class Surface **************************/
/***************************************************************************************/

void Surface::ImportCtrlPts(std::vector<Point3D> * ctrls, int fourctrl){ //  if fourctrl = 1 we copy only the four first control point (simulation of projection on a plane).

	if(fourctrl ==0){
		for(int i = 0; i<ctrls->size() ; i++){
			_Controls.push_back(Point3D((*ctrls)[i])) ;
		}
	}

	if(fourctrl){
		for(int i = 0; i<4 ; i++){
			_Controls.push_back(Point3D((*ctrls)[i])) ;
		}
	}

}

void Surface::ShowCtrlPts(){

	cout<<"show ctrl pts"<<endl;
	for(int i = 0; i<_Controls.size() ; i++){

		cout<<_Controls[i].x()<<" "<<_Controls[i].y()<<" "<<_Controls[i].z()<<endl;
	}
	cout<<endl;

}

void Surface::Proj3Dpts(){
  
  int control_size = _Controls.size();
  
  printf("Control point array size %d\n", control_size);
  
  if(control_size>3){

    double *e0 = diff(GetCtrls(2), GetCtrls(1));
    normalize(e0);
    double *e1 = diff(GetCtrls(0), GetCtrls(1));
    normalize(e1);

    
    for(int i = 0 ; i<control_size ; i++){
      AddCtrlIdx( \
	Point2D( \
	  double(_k)*prod_scal(e0, GetCtrls(i)), \
	  double(_k)*prod_scal(e1, GetCtrls(i)) 
	) \
      );
    }
  }
}


void Surface::GetEquations() {
	//create a delaunay triangulation
    Delaunay dt;
    dt.insert(_ControlsIdx.begin(), _ControlsIdx.end());

    //iterate through the faces
	int i = 0;
    Delaunay::Finite_faces_iterator it;
    for (it = dt.finite_faces_begin(); it != dt.finite_faces_end(); it++) // on ballaye tous les triangle de delaunay trouvé
    {
		//cout << "Face #" << i << ": " << dt.triangle(it)  << endl;
		i++;

		// Variable locale qui vont constituer "l'eqution de chaque triangle"
		double normal[3];
		double dist;
		double ctrls[3][3];
		int ctrlsidx[3][2];
		int idxPos[3];

		for (int k = 0; k < int(3); k++) 
		{
			ctrlsidx[k][0] = int(dt.triangle(it).vertex(k).x());// On recupere les index 2D des points des triangles
			ctrlsidx[k][1] = int(dt.triangle(it).vertex(k).y());
			int idx = -1;
			for (int l = 0; l < int(_ControlsIdx.size()); l++) // on cherche ici les points 3D correspondant
			{
				if (dt.triangle(it).vertex(k).x() == _ControlsIdx[l].x() && dt.triangle(it).vertex(k).y() == _ControlsIdx[l].y()) 
				{
					idx = l;
					break;
				}
			}

			ctrls[k][0] = _Controls[idx].x();// on recupere les points 3D
			ctrls[k][1] = _Controls[idx].y();
			ctrls[k][2] = _Controls[idx].z();
			
			idxPos[k] = idx;

		//	cout << "Vertex " << k << ": " << ctrls[k][0] << " " << ctrls[k][1] <<  " " << ctrls[k][2] << endl;
		}

		//	cout << "index " << idxPos[0] << ", " << idxPos[1] << ", " << idxPos[2] << endl;

		cross(normal, diff(ctrls[1],ctrls[0]), diff(ctrls[2],ctrls[0]));
		normalize(normal);
        dist = prod_scal(ctrls[0], normal);
		bool border[3] = {0, 0, 0};
		_Equations.push_back(Equations(normal, dist, /*ctrls, ctrlsidx, */idxPos, border));

    }
}


void Surface::FillEdges() {


	Eigen::Matrix2d inf;

	inf(0,0)=1.0;
	inf(0,1)=0.0;
	inf(1,0)=0.0;
	inf(1,1)=1.0;

	for(int i=0; i<int(this->_Equations.size()); i++){

	double ctrls[3][3];
	int ctrlsidx[3][2];
	int idx[3] = {_Equations[i]._idxPos[0], _Equations[i]._idxPos[1], _Equations[i]._idxPos[2]};


	// order index then from.idx < to.idx
	int a = idx[0], b = idx[1], c = idx[2];

	//if(a<b && a<c) { if(c<b){ idx[2] = b; idx[1] = c; }  else{ idx[2] = c; idx[1] = b; } } 

	//if(b<a && b<c) { if(c<a){ idx[2] = a; idx[1] = c; idx[0] = b; } else{ idx[1] = a; idx[0] = b; } }

	//if(c<a && c<b) { if(b<a){ idx[2] = a; idx[0] = c; } else{ idx[2] = b; idx[1] = a; idx[0] = c; } }


	AddEdge(idx[0], idx[1], idx[2], inf);
	AddEdge(idx[1], idx[2], idx[0], inf);
	AddEdge(idx[2], idx[0], idx[1], inf);


	}

}

void Surface::AddEdge(int i, int j, int k, Eigen::Matrix2d& inf){

	GridEdge edge;

	//double V1toBidx[2];
	double V1toB[3];

	//	base_2d(GetCtrlsIdx(i), GetCtrlsIdx(j), GetCtrlsIdx(k), V1toBidx);//1er arg : from, 2eme : to, 3eme : dernier pt
	base_3d(GetCtrls(i), GetCtrls(j), GetCtrls(k), V1toB);//1er arg : from, 2eme : to, 3eme : dernier pt

	edge.from = i;
	edge.to = j;
	edge.last = k;
	edge.trueMeas(0) = double(_k) * dist_3d(GetCtrls(i), GetCtrls(j));//3D dist
	edge.trueMeas(1) = double(_k) * sqrt( V1toB[0]*V1toB[0]	+ V1toB[1]*V1toB[1] + V1toB[2]*V1toB[2] );//3D height
	edge.information = inf;


	_EdgesMeasures.push_back(edge);
}


void Surface::g2omain()
{

  _OptControlsIdx.clear();

  init_tutorial_slam2d_types();

  /*********************************************************************************
   * creating the optimization problem
   ********************************************************************************/

  typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
  typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

  // allocating the optimizer
  SparseOptimizer optimizer;
  SlamLinearSolver* linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
  OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton(blockSolver);

  optimizer.setAlgorithm(solver);

  // adding the odometry to the optimizer
  // first adding all the vertices
  cerr << "Optimization: Adding robot poses ... ";

  for(int i=0; i < _ControlsIdx.size(); i++)
  {
    Vertex* indexVertx =  new Vertex;
    indexVertx->setId(i);
	indexVertx->setEstimate( GetControlsIdxValue(i) );
    optimizer.addVertex(indexVertx);
  }

  cerr << "done." << endl;



  // second add the odometry constraints
  cerr << "Optimization: Adding odometry measurements ... ";
    
  for (size_t i = 0; i < _EdgesMeasures.size(); ++i) {
	const GridEdge& simEdge = _EdgesMeasures[i];

    Edge* odometry = new Edge;
	const MeasType& meas = simEdge.trueMeas;


	odometry->vertices()[0] = optimizer.vertex(simEdge.from);
    odometry->vertices()[1] = optimizer.vertex(simEdge.to);
	//odometry->vertices().push_back(optimizer.vertex(simEdge.last));
    odometry->setVertex3( optimizer.vertex(simEdge.last) );
	odometry->setMeasurement(meas);
	odometry->setInformation(simEdge.information);

    optimizer.addEdge(odometry);
  }


  cerr << "done." << endl;


  /*********************************************************************************
   * optimization
   ********************************************************************************/

  // dump initial state to the disk
  optimizer.save("tutorial_before.g2o");

  // prepare and run the optimization
  // fix the control point :


//  {
//	Vertex* firstRobotPose = dynamic_cast<Vertex*>(optimizer.vertex(4));
//	firstRobotPose->setFixed(true);
//
//}


  optimizer.setVerbose(true);

  optimizer.save("tutorial_beforebis.g2o");


  cerr << "Optimizing" << endl;
  optimizer.initializeOptimization();
  optimizer.optimize(10);
  cerr << "done." << endl;

  optimizer.save("tutorial_after.g2o");

  for(int i = 0 ; i < _ControlsIdx.size() ; i++){

  Eigen::Vector2d Optimized2dPt;
  Optimized2dPt = dynamic_cast<Vertex*>(optimizer.vertex(i))->estimate();
  AddOptIdx(Point2D(Optimized2dPt[0],Optimized2dPt[1]));

  }




  // freeing the graph memory
  optimizer.clear();

  // destroy all the singletons
  Factory::destroy();
  OptimizationAlgorithmFactory::destroy();
  HyperGraphActionLibrary::destroy();

  }


  void Surface::OptiResults(){

	double distance = 0, rdist = 0, error=0, errorini=0, errorinisansbounding = 0, 
		errorsansbounding=0, fdist=0, mhigh = 0, rhigh=0, fhigh = 0;
  

	for(int i=0; i<_Controls.size() ; i++)

		{
			cout<<"Vertex n "<<i<<" :"<<_Controls[i].x()<<", "<<_Controls[i].y()<<", "<<_Controls[i].z()<<endl;

		}

  // Displaying the distance and error:

  /***/   ofstream fichier("tutorial_after.g2o", ios::app); 
  
  if(fichier)
  {
	fichier <<endl; 
  
	fichier<<"initial 3D Vertex : "<<endl;

	for(int i=0; i<_Controls.size() ; i++)

		{
			fichier<<"Vertex n "<<i<<" :"<<_Controls[i].x()<<", "<<_Controls[i].y()<<", "<<_Controls[i].z()<<endl;

		}
   
  
    fichier <<endl; 
	fichier<<"Displaying the distances: "<<endl;

  }


  for(int i=0 ; i < int(_EdgesMeasures.size()) ; i++)
  {

	int idx[3] ;
	int idx3d[3] ;

	idx[0] = _EdgesMeasures[i].from;
    idx[1] = _EdgesMeasures[i].to;
	idx[2] = _EdgesMeasures[i].last;


	distance = dist_2d(GetOptCtrlsIdx(idx[0]), GetOptCtrlsIdx(idx[1]));
	rdist = double(_k) * dist_3d(GetCtrls(idx[0]), GetCtrls(idx[1]));
	fdist = dist_2d(GetCtrlsIdx(idx[0]), GetCtrlsIdx(idx[1]));


	errorini += abs(rdist-fdist);
	error += abs(rdist-distance);


	//if( !( (from == 0 && to == 1) ||
	//(from == 1 && to == 2)  ||
	//(from == 2 && to == 3)  ||
	//(from == 0 && to == 3)     ) )

	//{

	//errorinisansbounding += abs(rdist-fdist);
	//errorsansbounding += abs(rdist-distance);

	//}

	if(fichier){
	fichier << "dist " << idx[0] <<"-"<< idx[1] <<" :   m : "<<distance<<" ; r : "<<rdist<<" ; f : "<<fdist<<endl;
	}

  }

	if(fichier){
		fichier<<endl;
		fichier<<"initial error : "<<errorini<<endl;
		fichier<<"total error : "<<error<<endl;
		//fichier<<"initial error sans bounding : "<<errorinisansbounding<<endl;
		//fichier<<"total error sans bounding : "<<errorsansbounding<<endl;
		fichier<<endl;
	}

	double V1toB[3];
	double V1toBidx[2];
	double V1toBidxOpti[2];

    for(int i=0 ; i < int(_EdgesMeasures.size()) ; i++)
  {

	int idx[3] ;
	int idx3d[3] ;

	idx[0] = _EdgesMeasures[i].from;
    idx[1] = _EdgesMeasures[i].to;
	idx[2] = _EdgesMeasures[i].last;

	base_2d(GetOptCtrlsIdx(idx[0]),GetOptCtrlsIdx(idx[1]),GetOptCtrlsIdx(idx[2]), V1toBidxOpti);
	base_3d(GetCtrls(idx[0]), GetCtrls(idx[1]), GetCtrls(idx[2]), V1toB);//1er arg : from, 2eme : to, 3eme : dernier pt
	base_2d(GetCtrlsIdx(idx[0]),GetCtrlsIdx(idx[1]),GetCtrlsIdx(idx[2]), V1toBidx);//1er arg : from, 2eme : to, 3eme : dernier pt

	mhigh =  sqrt(V1toBidxOpti[0]*V1toBidxOpti[0] + V1toBidxOpti[1]*V1toBidxOpti[1]) ;
	rhigh = double(_k) * sqrt( V1toB[0]*V1toB[0]+ V1toB[1]*V1toB[1] + V1toB[2]*V1toB[2] );
	fhigh = sqrt(V1toBidx[0]*V1toBidx[0] + V1toBidx[1]*V1toBidx[1]) ;

	if(fichier){
	fichier << "high " << idx[0] <<"-"<< idx[1]<<"-"<< idx[2] <<" :   m : "<<mhigh<<" ; r : "<<rhigh<<" ; f : "<<fhigh<<endl; 
	}

 }



  //displaying areas :

  if(fichier){fichier <<endl; fichier<<"Displaying areas: "<<endl; }

  double airemaxerror = 0, aireerror = 0;
  double SratioM = 0, SratioF = 0, ratiomin = 20;

  for(int i = 0; i<_Equations.size() ; i++)
  {
	double A[3], B[3], R[3];
	double A2[2], B2[2], A3[2], B3[2];
	double R2, R3;
	double aireR, aireF, aireM;
	double ratioM, ratioF;
	double nz;

	int idx[3] ;
	int idx3d[3] ;

	idx[0] = _EdgesMeasures[3*i].from;
	idx[1] = _EdgesMeasures[3*i].to;
	idx[2] = _EdgesMeasures[3*i].last;


	A[0] = _Controls[idx[1]].x() -  _Controls[idx[0]].x();
	A[1] = _Controls[idx[1]].y() -  _Controls[idx[0]].y();
	A[2] = _Controls[idx[1]].z() -  _Controls[idx[0]].z();

	B[0] = _Controls[idx[2]].x() -  _Controls[idx[0]].x();
	B[1] = _Controls[idx[2]].y() -  _Controls[idx[0]].y();
	B[2] = _Controls[idx[2]].z() -  _Controls[idx[0]].z();

	cross_3d(R, A, B);

	aireR= double(_k)*double(_k) * sqrt(R[0]*R[0] + R[1]*R[1] + R[2]*R[2]);


	A2[0] = _ControlsIdx[idx[1]].x() -  _ControlsIdx[idx[0]].x();
	A2[1] = _ControlsIdx[idx[1]].y() -  _ControlsIdx[idx[0]].y();

	B2[0] = _ControlsIdx[idx[2]].x() -  _ControlsIdx[idx[0]].x();
	B2[1] = _ControlsIdx[idx[2]].y() -  _ControlsIdx[idx[0]].y();

	R2 = cross_2d(R2, A2, B2);

	aireF = sqrt(R2*R2);


	A3[0] = _OptControlsIdx[idx[1]][0] -  _OptControlsIdx[idx[0]][0];
	A3[1] = _OptControlsIdx[idx[1]][1] -  _OptControlsIdx[idx[0]][1];

	B3[0] = _OptControlsIdx[idx[2]][0] -  _OptControlsIdx[idx[0]][0];
	B3[1] = _OptControlsIdx[idx[2]][1] -  _OptControlsIdx[idx[0]][1];

	R3 = cross_2d(R3, A3, B3);

	aireM = sqrt(R3*R3);

	aireerror += abs(aireM-aireR);

	if(abs(aireM-aireR)>airemaxerror){airemaxerror = abs(aireM-aireR);}


	ratioM=aireM/aireR;

	ratioF=/*2**/aireF/aireR;
  
	SratioM += ratioM;
	SratioF += ratioF;

	if(ratioM<ratiomin){ratiomin = ratioM;}

	nz = _Equations[i]._normal[2];


  if(fichier){
		fichier<<"aire tri n "<<i<<", "<<idx[0]<<" "<<idx[1]<<" "<<idx[2]<<" "<<" :  m : "<<aireM<<", r : "<<aireR<<", f : "<<aireF<<"  ,  ratio m : "<<ratioM<<"  ,  ratio f : "<<ratioF<<"  ,  normal.z : "<<nz<<endl;
  }

  }

   if(fichier){
  		fichier<<"erreur aire totale : "<<aireerror<<endl;
		fichier<<"erreur moyenne : "<<aireerror/_Equations.size()<<endl;
		fichier<<"max erreur d'aire : "<<airemaxerror<<endl;

		fichier<<"moyenne ratio m : "<<SratioM/_Equations.size()<<endl;
		fichier<<"min ratio m : "<<ratiomin<<endl;
		fichier<<"moyenne ratio f : "<<SratioF/_Equations.size()<<endl;

   }

}

int ptDupliTest(int pt, int nEdgemax, vector<edgeResult> EdgesResults){

	int numE = -1;

	for(int i = 0 ; i< EdgesResults.size() ; i++){

		if (EdgesResults[i].numTri != EdgesResults[nEdgemax].numTri  && 
				( pt == EdgesResults[i].idxPos[0] || pt == EdgesResults[i].idxPos[1] ) )
			{
				numE = i; 
		 		break;			
			}
	}

	return numE;
}

int nbBordsTest(int numTri,vector<edgeResult> EdgesResults){

	int result = 0;
	for(int i = 0 ; i< EdgesResults.size() ; i++){

		if (EdgesResults[i].numTri == EdgesResults[numTri].numTri) result++;
	}

	return result;
}


bool Surface::findPointToDupli(){

	vector<edgeResult> EdgesResults;
	edgeResult edgeRes;
	int idx[3] = {-1, -1, -1} ;
	double V1toBidxOpti[2];
	double V1toB[3];
	double maxf = 0.0;
	int nEdgemax = -1;
	int j = 0, l = 0;

	for (int i=0; i<_Equations.size() ; i++){

		if(TriangleBorderTest(i)){
			
			idx[0] = _Equations[i]._idxPos[0]; 	idx[1] = _Equations[i]._idxPos[1]; 	idx[2] = _Equations[i]._idxPos[2]; 

			for(int k = 0; k<3 ; k++){// k - j - l 3 position dans les pts du triangles
				
				if(_Equations[i]._border[k]){

						edgeRes.numTri = i;

						edgeRes.idxPos[0] = idx[k];

						if(k==0) {j=1; l=2;}
						if(k==1) {j=2; l=0;}	
						if(k==2) {j=0; l=1;}

						edgeRes.idxPos[1] = idx[j];

						//cout<<k<<" , "<<j<<endl;
						
						edgeRes.res[0] = sqrt( pow(GetOptCtrlsIdx(idx[k])[0]-GetOptCtrlsIdx(idx[j])[0],2) + pow(GetOptCtrlsIdx(idx[k])[1]-GetOptCtrlsIdx(idx[j])[1],2) );
						edgeRes.res[1] = _k * sqrt( pow(GetCtrls(idx[k])[0]-GetCtrls(idx[j])[0],2) + pow(GetCtrls(idx[k])[1]-GetCtrls(idx[j])[1],2) + pow(GetCtrls(idx[k])[2]-GetCtrls(idx[j])[2],2) );
						
						base_2d(GetOptCtrlsIdx(idx[j]), GetOptCtrlsIdx(idx[l]), GetOptCtrlsIdx(idx[k]), V1toBidxOpti); 
						base_3d(GetCtrls(idx[j]), GetCtrls(idx[l]), GetCtrls(idx[k]), V1toB);
						edgeRes.res[2]  =  sqrt(V1toBidxOpti[0]*V1toBidxOpti[0] + V1toBidxOpti[1]*V1toBidxOpti[1]) ;
						edgeRes.res[3]  = double(_k) * sqrt( V1toB[0]*V1toB[0]+ V1toB[1]*V1toB[1] + V1toB[2]*V1toB[2] );
						edgeRes.res[4]  = double(edgeRes.res[0]/edgeRes.res[1]) + double(edgeRes.res[3]/edgeRes.res[2]);

						if(edgeRes.res[4]> maxf)  
						{
							maxf = edgeRes.res[4]; 
							nEdgemax = EdgesResults.size();
						}

						
						EdgesResults.push_back(edgeRes);

				}
			}
		}
	}


	//cout<<"EdgesResults "<<nEdgemax<<endl;

	//for(int i=0; i<EdgesResults.size() ; i++){

	//	cout<<"numTri : "<<EdgesResults[i].numTri<<",  "<<EdgesResults[i].idxPos[0] <<"  "<<EdgesResults[i].idxPos[1] <<" , dopt : "<<EdgesResults[i].res[0] <<" , dreal : "<<EdgesResults[i].res[1] <<" , hopt : "<<EdgesResults[i].res[2] <<" , hreal : "<<EdgesResults[i].res[3] <<" , f : "<<EdgesResults[i].res[4]  <<endl;

	//}


	if(maxf>2.1 && double(EdgesResults[nEdgemax].res[0]/EdgesResults[nEdgemax].res[1])>1 && double(EdgesResults[nEdgemax].res[3]/EdgesResults[nEdgemax].res[2])>1 ){

		// Les deux points sont ils connecté à daute triangle au bord ?
		int pt1 = EdgesResults[nEdgemax].idxPos[0];
		int pt2 = EdgesResults[nEdgemax].idxPos[1];
		int pt = -1;
		int numE1 = ptDupliTest(pt1, nEdgemax, EdgesResults);//permet de savoir si le point pt1 apparait dans un autre triangle qui a un edge au bord
		int numE2 = ptDupliTest(pt2, nEdgemax, EdgesResults);
		double f1 = 0 ; if(numE1 != -1){ f1 = EdgesResults[numE1].res[4];}
		double f2 = 0 ; if(numE2 != -1){ f2 = EdgesResults[numE2].res[4];}

		//cout<<"num Tri "<<EdgesResults[nEdgemax].numTri<<endl;
		//cout<<"pts : "<<pt1<<" , "<<pt2<<endl;
		//cout<<"f1 : "<<f1<<endl;
		//cout<<"f2 : "<<f2<<endl;
		//cout<<"max f "<<maxf<<endl;
	

		// 3 cases
		//if(numE1 == -1 && numE2 == -1)
		//{ 
	
		//} //pas dedoublable

		//if(numE1 != -1 && numE2 == -1){ 
	
	
		//} // cas ou on pourrait dedoubler avec un autre triangle... 
		//					  // Mais si on dedouble que ce traingle il ne lui restera qu'un pt accroché au graphe

		//if(numE1 == -1 && numE2 != -1){ 
	
	
		//} // de même

		//// is numE1 ou numE2 = 0 on pourrai imaginer faire la procedure avec le suivant triangle qui a fmax>2.1 et tester si les deux nouveaux E1 et E2 sont != -1

		if(numE1 != -1 && numE2 != -1 && nbBordsTest(nEdgemax, EdgesResults) < 2) /*pas sur que cette derniere condition soit utile vu que l'on regarde si numE1/2 existe*/ 
																					// duplicable, il faut choisir le point
		{
	//		cout<<"nbBordsTest : "<<nbBordsTest(numE1, EdgesResults)<<" "<<nbBordsTest(numE2, EdgesResults)<<endl;
			if(nbBordsTest(numE1, EdgesResults) < 2 && nbBordsTest(numE2, EdgesResults) < 2){//we also check of the barder triangle have less than 2 edges on the border
				if(f2>f1) pt = pt2; //we compare result of the fonctions f on the two border, 
				else pt = pt1;
			}
			if(!(nbBordsTest(numE1, EdgesResults) < 2)  && nbBordsTest(numE2, EdgesResults) < 2 ) pt = pt2; //we compare result of the fonctions f on the two border, 
			if(nbBordsTest(numE1, EdgesResults) < 2  && !(nbBordsTest(numE2, EdgesResults) < 2) ) pt = pt1;//we also check of the barder triangle have less than 2 edges on the border
		}

		if ( pt != -1 ){
			dupliIndex(pt, EdgesResults[nEdgemax].numTri);
		//	cout<<"ok dupli : "<<pt<<" "<<EdgesResults[nEdgemax].numTri<<endl;
			_numLastInxDupli = pt;
			return true;
		}
	}

	return false;
}



bool twoIdxMatch(int idx0, int idx1, int idxref0, int idxref1){

	if ( (idx0 == idxref0 && idx1 == idxref1) || (idx0 == idxref1 && idx1 == idxref0) )
		return true;

	else 
	return false;
}


bool two3IdxMatch(int idx0, int idx1, int idx2, int idxref0, int idxref1){

	if ( twoIdxMatch(idx0, idx1, idxref0, idxref1) || 
		 twoIdxMatch(idx1, idx2, idxref0, idxref1)  || 
		 twoIdxMatch(idx2, idx0, idxref0, idxref1)  )
		return true;

	else 
	return false;
}


bool Surface::TriangleBorderTest(int numTri){

	int idxPos[3];

	idxPos[0] = _Equations[numTri]._idxPos[0];
	idxPos[1] = _Equations[numTri]._idxPos[1];
	idxPos[2] = _Equations[numTri]._idxPos[2];

	bool test0=0, test1=0, test2=0; 

	for(int i=0; i<_Equations.size(); i++){
		if(i != numTri){

			if(	two3IdxMatch(_Equations[i]._idxPos[0], _Equations[i]._idxPos[1], _Equations[i]._idxPos[2], idxPos[0], idxPos[1]) )
			test0 = 1;

			if(	two3IdxMatch(_Equations[i]._idxPos[0], _Equations[i]._idxPos[1], _Equations[i]._idxPos[2], idxPos[1], idxPos[2]) )
			test1 = 1;

			if(	two3IdxMatch(_Equations[i]._idxPos[0], _Equations[i]._idxPos[1], _Equations[i]._idxPos[2], idxPos[2], idxPos[0]) )
			test2 = 1;

		}
	}

	if(test0==0){_Equations[numTri]._border[0] = 1;}

	if(test1==0){_Equations[numTri]._border[1] = 1;}

	if(test2==0){_Equations[numTri]._border[2] = 1;}


	if( (test0==0) || (test1==0) || (test2==0) )
	return true;

	else return false;
}


void Surface::dupliIndex(int numDupli, int numTri){

_ControlsIdx.push_back(_ControlsIdx[numDupli]);
_Controls.push_back(_Controls[numDupli]);


for(int k=0; k<3; k++){
	if(_EdgesMeasures[3*numTri+k].to == numDupli){ _EdgesMeasures[3*numTri+k].to = _ControlsIdx.size()-1 ;}
	if(_EdgesMeasures[3*numTri+k].from == numDupli){ _EdgesMeasures[3*numTri+k].from = _ControlsIdx.size()-1 ;}
	if(_EdgesMeasures[3*numTri+k].last == numDupli){ _EdgesMeasures[3*numTri+k].last = _ControlsIdx.size()-1 ;}
}


for(int k=0; k<3; k++){
	if(_Equations[numTri]._idxPos[k] == numDupli){ _Equations[numTri]._idxPos[k] = _ControlsIdx.size()-1 ;}
}

}



bool intersecTest(double A[2], double B[2], double C[2], double D[2]){

	double k = -1;// 2 parameters
	double m = -1;// describing segments


	double I[2]={B[0]-A[0], B[1]-A[1]};
	double J[2]={D[0]-C[0], D[1]-C[1]};

	// P = A + k *(B-A) = C+ m *(D-C)
	//k Ix - m Jx=Cx-Ax = C1
	//k Iy - m Jy=Cy-Ay = C2

	double D11 = I[0], D21 = I[1], D12 = -J[0], D22 = -J[1] ; //ex. YB1 = B1[1]
	double C1 = C[0]-A[0] , C2 = C[1]-A[1];

	double Det = D11*D22-D21*D12; /*J[0]*I[1]-I[0]*J[1];*/
	double Detk, Detm;

	if(Det !=0)
	{
		Detk = C1*D22-C2*D12 ; k = Detk/Det;
		Detm = D11*C2-D21*C1 ; m = Detm/Det;

		if(k>0+EPSILON && k<1-EPSILON && m>0+EPSILON && m<1-EPSILON ){ 
				//cout<<" D11:"<<D11<<" D21:"<<D21<<" D12:"<<D12<<" D22:"<<D22<<" C1:"<<C1<<" C2:"<<C2<<endl;
				//cout<<" A[0]:"<<A[0]<<" A[1]:"<<A[1]<<" B[0]:"<<B[0]<<" B[1]:"<<B[1]<<" C[0]:"<<C[0]<<" C[1]:"<<C[1]<<" D[0]:"<<D[0]<<" D[1]:"<<D[1]<<endl;
				//cout<<"Det :"<<Det<<endl;
				//cout<<"Detk :"<<Detk<<endl;
				//cout<<"Detm :"<<Detm<<endl;
				//cout<<"k :"<<k<<endl;
				//cout<<"m :"<<m<<endl;
				//cout<<"intersecte"<<endl;
				return true;
		}
	
	}

	//cout<<" D11:"<<D11<<" D21:"<<D21<<" D12:"<<D12<<" D22:"<<D22<<" C1:"<<C1<<" C2:"<<C2<<endl;
	//cout<<" A[0]:"<<A[0]<<" A[1]:"<<A[1]<<" B[0]:"<<B[0]<<" B[1]:"<<B[1]<<" C[0]:"<<C[0]<<" C[1]:"<<C[1]<<" D[0]:"<<D[0]<<" D[1]:"<<D[1]<<endl;
	//cout<<"Det :"<<Det<<endl;
	//cout<<"Detk :"<<Detk<<endl;
	//cout<<"Detm :"<<Detm<<endl;
	//cout<<"k :"<<k<<endl;
	//cout<<"m :"<<m<<endl;


	return false;
}


bool overlap1SegTest(int numTri, int id1, int id2, vector <Equations> _Equations, vector <Point2D> _OptControlsIdx){

double A[2], B[2], C[2], D[2]; int idnew[2];
A[0] = _OptControlsIdx[_Equations[numTri]._idxPos[id1]][0];
A[1] = _OptControlsIdx[_Equations[numTri]._idxPos[id1]][1];

B[0] = _OptControlsIdx[_Equations[numTri]._idxPos[id2]][0];
B[1] = _OptControlsIdx[_Equations[numTri]._idxPos[id2]][1];

		for(int i = 0 ; i < _Equations.size() ; i++){
			if(i != numTri){
				for(idnew[0]=0 ; idnew[0] < 2 ; idnew[0] ++){
					if(idnew[0]==0) idnew[1]=1;	if(idnew[0]==1) idnew[1]=2;	if(idnew[0]==2) idnew[1]=0;

					C[0] = _OptControlsIdx[_Equations[i]._idxPos[idnew[0]]][0];
					C[1] = _OptControlsIdx[_Equations[i]._idxPos[idnew[0]]][1];

					D[0] = _OptControlsIdx[_Equations[i]._idxPos[idnew[1]]][0];
					D[1] = _OptControlsIdx[_Equations[i]._idxPos[idnew[1]]][1];

					if( intersecTest(A,B,C,D) )
						 {	cout<<"overlap ! : "<<i<<" "<<idnew[0]<<" "<<idnew[1]<<" "<<_Equations[i]._idxPos[idnew[0]]<<" "<<_Equations[i]._idxPos[idnew[1]]<<endl;
							return true;
						 }

				}	
			}	

		}
		return false;
}

bool Surface::OverlapTest(){

	int id[2];

	Eigen::Vector2d A;

	for(int i = 0 ; i < _Equations.size() ; i++){

		for(id[0]=0 ; id[0] < 2 ; id[0] ++){
			if(id[0]==0) id[1]=1;	if(id[0]==1) id[1]=2;	if(id[0]==2) id[1]=0;
			
			if(overlap1SegTest(i, id[0], id[1], _Equations, _OptControlsIdx)){	
				cout<<"overlap ! : "<<i<<" "<<id[0]<<" "<<id[1]<<" "<<_Equations[i]._idxPos[id[0]]<<" "<<_Equations[i]._idxPos[id[1]]<<endl;
				return true;}
		}
	}

	cout<<"No overlap"<<endl;
	return false;
}


void Surface::OptCopytoBuff(){
		_OptCtrsBuff.clear();
	for(int i=0 ; i<_OptControlsIdx.size() ; i++){
		_OptCtrsBuff.push_back(Point2D(_OptControlsIdx[i][0], _OptControlsIdx[i][1]));
	}
}

void Surface::RecoverFromBuff(){// dans cette fonction, non seulement on recharge les données de buffOpt dans la liste indexOpt, mais on remet toute les données (Equations, ControlsIdx, Controls et Edges) de maniere à ce que un g2omain() redonne bien la même optimisation

	int lastpos = _ControlsIdx.size()-1;// on va chercher le drenier point ajouté et le remplacer par le num du dernier point dupliqué

	for(int i =0; i<_Equations.size() ; i++){
		for(int k = 0; k< 3; k++){
			if(_Equations[i]._idxPos[k] == lastpos)
				{
						_Equations[i]._idxPos[k] = _numLastInxDupli;
				}
		}
	}

	for(int i =0; i<_EdgesMeasures.size() ; i++){
		if(_EdgesMeasures[i].from == lastpos){_EdgesMeasures[i].from =_numLastInxDupli;}
		if(_EdgesMeasures[i].to == lastpos){_EdgesMeasures[i].to =_numLastInxDupli;}
		if(_EdgesMeasures[i].last == lastpos){_EdgesMeasures[i].last =_numLastInxDupli;}
	}

	//_Controls.erase( --(_Controls.end()) ) ;

	_Controls.pop_back();
	_ControlsIdx.pop_back();

	_OptControlsIdx.clear();
	for(int i=0 ; i<_OptCtrsBuff.size() ; i++){
		_OptControlsIdx.push_back( Point2D(_OptCtrsBuff[i][0],_OptCtrsBuff[i][1] ));
	}
	_OptCtrsBuff.clear();

}

void Surface::ClearOptBuff(){
	if(_OptCtrsBuff.size()>0){
			_OptCtrsBuff.clear();
	}
}


void Surface::ShiftOptCtrlsIdx(){

	double minx = DBL_MAX, miny=DBL_MAX;


	for(int i=0; i<_OptControlsIdx.size(); i++){
	//		cout<<_OptControlsIdx[i][0]<<" "<<_OptControlsIdx[i][1]<<endl;
			if(_OptControlsIdx[i][0]<minx) minx = _OptControlsIdx[i][0];
			if(_OptControlsIdx[i][1]<miny) miny = _OptControlsIdx[i][1];
	}

	//cout<<"min : "<<minx<<" "<<miny<<endl;

	for(int i=0; i<_OptControlsIdx.size(); i++){
			_OptControlsIdx[i] = Point2D(_OptControlsIdx[i][0] - minx, _OptControlsIdx[i][1] - miny );
		//	cout<<_OptControlsIdx[i][0]<<" "<<_OptControlsIdx[i][1]<<endl;
	}

		//Determine the size of the Image :

	double maxx = DBL_MIN, maxy = DBL_MIN;
	for(int i=0; i<_OptControlsIdx.size(); i++){
			if(_OptControlsIdx[i][0]> maxx) maxx = _OptControlsIdx[i][0];
			if(_OptControlsIdx[i][1]> maxy) maxy = _OptControlsIdx[i][1];
	}

	_n = int(maxx) + 2;
	_m = int(maxy) + 2;

	//cout<<"size "<<_n<<" "<<_m<<endl;

}


void Surface::ComputeImgIdx() {



	int nbPlan = _Equations.size();
    _ImgIndx = (int **) malloc(_n*sizeof(int *));
    
    for (int i=0; i < _n ; i++) {
		_ImgIndx[i] = (int *) malloc(_m*sizeof(int));
        for (int j=0; j < _m; j++) {
			_ImgIndx[i][j] = -1;
			double proj[3] = {double(i), double(j), 0.0};
            for (int k=0; k < nbPlan; k++) {      
				double a[3] = {double(GetOptCtrlsIdx(_Equations[k]._idxPos[0])[0]), double(GetOptCtrlsIdx(_Equations[k]._idxPos[0])[1]), 0.0};
				double b[3] = {double(GetOptCtrlsIdx(_Equations[k]._idxPos[1])[0]), double(GetOptCtrlsIdx(_Equations[k]._idxPos[1])[1]), 0.0};
				double c[3] = {double(GetOptCtrlsIdx(_Equations[k]._idxPos[2])[0]), double(GetOptCtrlsIdx(_Equations[k]._idxPos[2])[1]), 0.0};
				double ref[3];
	/*			double res1[3], res2[3];*/
				cross(ref, diff(b,a), diff(c,a));
				double tmp[3];
				cross(tmp, diff(a,proj), diff(b,proj));
                double test1 = prod_scal(ref,tmp);
				cross(tmp, diff(b,proj), diff(c,proj));
                double test2 = prod_scal(ref,tmp);
				cross(tmp, diff(c,proj), diff(a,proj));
                double test3 = prod_scal(ref,tmp);

                if (test1 >= 0 && test2 >= 0 && test3 >= 0) {// test if "proj" is in the triangle abc
                    _ImgIndx[i][j] = k;
					//cout << k << " ";
                    break;
				}
			}
		}
		//cout << endl;
	}

}


void Surface::ComputeBumpImg(std::vector<cv::Point3d >& blob) {

	//allocation et init du bump img :
    _BumpImg = (double **) malloc(_n*sizeof(double *));
   
    for (int i=0; i < _n ; i++) {
		_BumpImg[i] = (double *) malloc(_m*sizeof(double));
        for (int j=0; j < _m; j++) {
			_BumpImg[i][j] = -1;
		}
	}


	for(int i= 0 ; i<blob.size() ; i++){

		double disProj = -1;
		int idx[2] = {-1, -1};
		int numTri = -1;

		if(GetDistAndIdx(blob[i], disProj, idx, numTri) ){// on peut projeter le point

			_BumpImg[idx[0]][idx[1]] = disProj;


			//cout<<"numTri : "<<numTri<<endl;
			//cout<<"idx : "<<idx[0]<<" "<<idx[1]<<endl;
			//cout<<"dist Proj : "<<disProj<<endl;

		}

		else cout<<"proj pas possible"<<endl;

	}

}

void getIndex2D(double pos_3[3], const double * ctrls0, const double * ctrls1, const double * ctrls2, const double * ctrlsidx0, const double * ctrlsidx1, const double * ctrlsidx2, int res[2]) {
    // Compute percentage of ctrl point 1
	double *A = diff(ctrls1, ctrls2);
	normalize(A);
	
	double *B = diff(pos_3, ctrls0);
	normalize(B);
    
	double num, den, lambda;
    if (B[0] != 0.0) {
        num = double(ctrls0[1] - ctrls2[1]) + double(ctrls2[0] - ctrls0[0])*(B[1]/B[0]);
        den = A[1] - A[0]*(B[1]/B[0]);
	} else {
        num = double(ctrls0[0] - ctrls2[0]);
        den = A[0];
	}
    
    if (den != 0.0) {
        lambda = num/den;
	}
    
	double inter_pos [3];
    inter_pos [0] = ctrls2[0] + lambda*A[0];
    inter_pos [1] = ctrls2[1] + lambda*A[1];
    inter_pos [2] = ctrls2[2] + lambda*A[2];
    
    A = diff(inter_pos, ctrls0);  
    double val = sqrt(prod_scal(A, A));
    A = diff(inter_pos, pos_3); 
	double u = sqrt(prod_scal(A, A));
    u = u / val;

    // Compute percentage of ctrl point 2
	A = diff(ctrls0, ctrls2);
	normalize(A);
	
	B = diff(pos_3, ctrls1);
	normalize(B);
    
    if (B[0] != 0.0) {
        num = double(ctrls1[1] - ctrls2[1]) + double(ctrls2[0] - ctrls1[0])*(B[1]/B[0]);
        den = A[1] - A[0]*(B[1]/B[0]);
	} else {
        num = double(ctrls1[0] - ctrls2[0]);
        den = A[0];
	}
    
    if (den != 0.0) {
        lambda = num/den;
	}
    
    inter_pos [0] = ctrls2[0] + lambda*A[0];
    inter_pos [1] = ctrls2[1] + lambda*A[1];
    inter_pos [2] = ctrls2[2] + lambda*A[2];
	

    A = diff(inter_pos, ctrls1);  
    val = sqrt(prod_scal(A, A));
    A = diff(inter_pos, pos_3); 
	double v = sqrt(prod_scal(A, A));
    v = v / val;
    
    // Compute percentage of ctrl point 3
	A = diff(ctrls0, ctrls1);
	normalize(A);
	
	B = diff(pos_3, ctrls2);
	normalize(B);
    
    if (B[0] != 0.0) {
        num = double(ctrls2[1] - ctrls1[1]) + double(ctrls1[0] - ctrls2[0])*(B[1]/B[0]);
        den = A[1] - A[0]*(B[1]/B[0]);
	} else {
        num = double(ctrls2[0] - ctrls1[0]);
        den = A[0];
	}
    
    if (den != 0.0) {
        lambda = num/den;
	}
    
    inter_pos [0] = ctrls1[0] + lambda*A[0];
    inter_pos [1] = ctrls1[1] + lambda*A[1];
    inter_pos [2] = ctrls1[2] + lambda*A[2];

    
    A = diff(inter_pos, ctrls2);  
    val = sqrt(prod_scal(A, A));
    A = diff(inter_pos, pos_3); 
	double w = sqrt(prod_scal(A, A));
    w = w / val;
        
	res[0] = int((u*ctrlsidx0[0] + v*ctrlsidx1[0] + w*ctrlsidx2[0])/(u + v + w));
	res[1] = int((u*ctrlsidx0[1] + v*ctrlsidx1[1] + w*ctrlsidx2[1])/(u + v + w));

}


bool Surface::GetDistAndIdx(cv::Point3d pt, double& distProj, int idx[2], int& numTri) {

	int nbPlan = _Equations.size();
	double Inpt[3];
	Inpt[0] = pt.x; Inpt[1] = pt.y; Inpt[2] = pt.z;
    
    double dmin = DBL_MAX;
	int ind_res[2] = {-1,-1};
	int numTrimin = -1;
    
    for (int i = 0; i < nbPlan; i++) {

		double error_dist = /*abs(*/prod_scal(Inpt, _Equations[i]._normal) - _Equations[i]._dist;// valeur absolue de la distance au plan
        
		double proj[3];
		proj[0] = Inpt[0] - error_dist*_Equations[i]._normal[0];
		proj[1] = Inpt[1] - error_dist*_Equations[i]._normal[1];
		proj[2] = Inpt[2] - error_dist*_Equations[i]._normal[2];
		
		double ref[3];
		cross(ref, diff(GetCtrls(_Equations[i]._idxPos[1]), GetCtrls(_Equations[i]._idxPos[0])),diff(GetCtrls(_Equations[i]._idxPos[2]),GetCtrls(_Equations[i]._idxPos[0])));
		double tmp [3];
		cross(tmp, diff(GetCtrls(_Equations[i]._idxPos[0]),proj),diff(GetCtrls(_Equations[i]._idxPos[1]),proj));
        double test1 = prod_scal(ref, tmp);
		cross(tmp, diff(GetCtrls(_Equations[i]._idxPos[1]),proj),diff(GetCtrls(_Equations[i]._idxPos[2]),proj));
        double test2 = prod_scal(ref, tmp); 
		cross(tmp, diff(GetCtrls(_Equations[i]._idxPos[2]),proj),diff(GetCtrls(_Equations[i]._idxPos[0]),proj));
        double test3 = prod_scal(ref, tmp);
		                
        if (error_dist < dmin && test1 >= 0.0 && test2 >= 0.0 && test3 >= 0.0) {// on cherche le plan qui est le plus près et qur lequel on peut faire la projection
            dmin = error_dist;
            numTrimin = i;
            getIndex2D(proj,GetCtrls(_Equations[i]._idxPos[0]), 
							GetCtrls(_Equations[i]._idxPos[1]), 
							GetCtrls(_Equations[i]._idxPos[2]), 
							GetOptCtrlsIdx(_Equations[i]._idxPos[0]), 
							GetOptCtrlsIdx(_Equations[i]._idxPos[1]), 
							GetOptCtrlsIdx(_Equations[i]._idxPos[2]), ind_res);
		}
	}
    
    if (numTrimin != -1) {
		idx[0] = ind_res[0];
		idx[1] = ind_res[1];
		distProj = dmin;
		numTri = numTrimin;
	
	cout<<endl;
	//cout<<"index nouvaux point : "<<numTrimin<<endl;
	//cout<<"nouveau point 3D : "<<Inpt[0]<<" , "<<Inpt[1]<<" , "<<Inpt[2]<<endl;
	//cout<<"index 2D : "<<ind_res[0]<<" , "<<ind_res[1]<<";"<<endl;
	return true;
	}

	if (numTrimin == -1){
		return false;
	}

}


void Surface::DrawSurf(){
	_VtxImg = (double ***) malloc(_n*sizeof(double **));
    
    for (int i=0; i < _n; i++) {
		_VtxImg[i] = (double **) malloc(_m*sizeof(double *));
        for (int j=0; j < _m; j++) {        
			_VtxImg[i][j] = NULL;
            if (_ImgIndx[i][j] > -1) {
                double *pt = getPoint3D(i,j,_ImgIndx[i][j]);
				_VtxImg[i][j] = (double *) malloc(3*sizeof(double));
				_VtxImg[i][j][0] = pt[0];
				_VtxImg[i][j][1] = pt[1];
				_VtxImg[i][j][2] = pt[2];
				delete pt;
			}
		}
	}	
}




void Surface::DrawRecImg(){
	_RecImg = (double ***) malloc(_n*sizeof(double **));
    
    for (int i=0; i < _n; i++) {
		_RecImg[i] = (double **) malloc(_m*sizeof(double *));
        for (int j=0; j < _m; j++) {        
			_RecImg[i][j] = NULL;
            if (_BumpImg[i][j] != -1 && _ImgIndx[i][j] != -1) {
				double *pt = getPoint3D(i,j,_ImgIndx[i][j]);
				_RecImg[i][j] = (double *) malloc(3*sizeof(double));
				_RecImg[i][j][0] = pt[0] + _BumpImg[i][j] * _Equations[_ImgIndx[i][j]]._normal[0];
				_RecImg[i][j][1] = pt[1] + _BumpImg[i][j] * _Equations[_ImgIndx[i][j]]._normal[1];
				_RecImg[i][j][2] = pt[2] + _BumpImg[i][j] * _Equations[_ImgIndx[i][j]]._normal[2];
				delete pt;

			}
		}
	}	
}


double* Surface::getPoint3D(int i, int j, int numPlan) {
	int pos_2[2] = {i, j};

	int P0[2] = { int(GetOptCtrlsIdx(_Equations[numPlan]._idxPos[0])[0]), int(GetOptCtrlsIdx(_Equations[numPlan]._idxPos[0])[1]) };
	int P1[2] = { int(GetOptCtrlsIdx(_Equations[numPlan]._idxPos[1])[0]), int(GetOptCtrlsIdx(_Equations[numPlan]._idxPos[1])[1]) };
	int P2[2] = { int(GetOptCtrlsIdx(_Equations[numPlan]._idxPos[2])[0]), int(GetOptCtrlsIdx(_Equations[numPlan]._idxPos[2])[1]) };
	
	const double* P3d0;
	const double* P3d1;
	const double* P3d2;
	
	P3d0 = GetCtrls(_Equations[numPlan]._idxPos[0]);
	P3d1 = GetCtrls(_Equations[numPlan]._idxPos[1]);
	P3d2 = GetCtrls(_Equations[numPlan]._idxPos[2]);


    // Compute percentage of ctrl point 1
	double *A = diff(P1, P2);
	normalize_2(A);
	
	double *B = diff(pos_2, P0);
	normalize_2(B);
    
	double num, den, lambda;
    if (B[0] != 0.0) {
        num = double(P0[1] - P2[1]) + double(P2[0] - P0[0])*(B[1]/B[0]);
        den = A[1] - A[0]*(B[1]/B[0]);
	} else {
        num = double(P0[0] - P2[0]);
        den = A[0];
	}
    
    if (den != 0.0) {
        lambda = num/den;
	}
    
	int inter_pos [2];
    inter_pos [0] = int(P2[0] + lambda*A[0]);
    inter_pos [1] = int(P2[1] + lambda*A[1]);

    
    A = diff(inter_pos, P0);  
    double val = sqrt(prod_scal_2(A, A));
    A = diff(inter_pos, pos_2); 
	double u = sqrt(prod_scal_2(A, A));
    u = u / val;

    // Compute percentage of ctrl point 2
	A = diff(P0, P2);
	normalize_2(A);
	
	B = diff(pos_2, P1);
	normalize_2(B);
    
    if (B[0] != 0.0) {
        num = double(P1[1] - P2[1]) + double(P2[0] - P1[0])*(B[1]/B[0]);
        den = A[1] - A[0]*(B[1]/B[0]);
	} else {
        num = double(P1[0] - P2[0]);
        den = A[0];
	}
    
    if (den != 0.0) {
        lambda = num/den;
	}
    
    inter_pos [0] = int(P2[0] + lambda*A[0]);
    inter_pos [1] = int(P2[1] + lambda*A[1]);
	

    A = diff(inter_pos, P1);  
    val = sqrt(prod_scal_2(A, A));
    A = diff(inter_pos, pos_2); 
	double v = sqrt(prod_scal_2(A, A));
    v = v / val;
    
    // Compute percentage of ctrl point 3
	A = diff(P0, P1);
	normalize_2(A);
	
	B = diff(pos_2, P2);
	normalize_2(B);
    
    if (B[0] != 0.0) {
        num = double(P2[1] - P1[1]) + double(P1[0] - P2[0])*(B[1]/B[0]);
        den = A[1] - A[0]*(B[1]/B[0]);
	} else {
        num = double(P2[0] - P1[0]);
        den = A[0];
	}
    
    if (den != 0.0) {
        lambda = num/den;
	}
    
    inter_pos [0] = int(P1[0] + lambda*A[0]);
    inter_pos [1] = int(P1[1] + lambda*A[1]);

    
    A = diff(inter_pos, P2);  
    val = sqrt(prod_scal_2(A, A));
    A = diff(inter_pos, pos_2); 
	double w = sqrt(prod_scal_2(A, A));
    w = w / val;
        
	double *res = new double[3];
	res[0] = (u*P3d0[0] + v*P3d1[0] + w*P3d2[0])/(u + v + w);
	res[1] = (u*P3d0[1] + v*P3d1[1] + w*P3d2[1])/(u + v + w);
	res[2] = (u*P3d0[2] + v*P3d1[2] + w*P3d2[2])/(u + v + w);

	return res;

}

void Surface::DisplayLabelImg(string filename_label){
	//Display label image :

	cv::Mat IdxImg(_m, _n, CV_8UC3);

	for (int i = 0; i < _n; i++) {
		for (int j = 0; j < _m; j++) {
			if ( _ImgIndx[i][j] != -1) {
				IdxImg.at<cv::Vec3b>(_m-1-j,i)[0] = 255;
				IdxImg.at<cv::Vec3b>(_m-1-j,i)[1] = 255;
				IdxImg.at<cv::Vec3b>(_m-1-j,i)[2] = 255;
			} 
			else {
				IdxImg.at<cv::Vec3b>(_m-1-j,i)[0] = 0;
				IdxImg.at<cv::Vec3b>(_m-1-j,i)[1] = 0;
				IdxImg.at<cv::Vec3b>(_m-1-j,i)[2] = 0;
			}
		}
	}


	for(int k=0; k < _Equations.size(); k++) {
	//unsigned char R = unsigned char((float(rand())/RAND_MAX)*255.0);
	//unsigned char G = unsigned char((float(rand())/RAND_MAX)*255.0);
	//unsigned char B = unsigned char((float(rand())/RAND_MAX)*255.0);

//	glColor4ub(500*((i+2)*(i+1)),200*(i+1)*(i+5),300*(i+1), 80);

	unsigned char R = (unsigned char)(500*((k+2)*(k+1)));
	unsigned char G = (unsigned char)(200*(k+1)*(k+5));
	unsigned char B = (unsigned char)(300*(k+1));

		for(int i = 0; i<_n; i++){
			for(int j = 0; j<_m ; j++){

				if(_ImgIndx[i][j] == k){
				IdxImg.at<cv::Vec3b>(_m-1-j,i)[2] = R;
				IdxImg.at<cv::Vec3b>(_m-1-j,i)[1] = G;
				IdxImg.at<cv::Vec3b>(_m-1-j,i)[0] = B;

				}
			}	
		}
	}

	cv::imshow(filename_label, IdxImg);
	cv::imwrite(filename_label, IdxImg);


}

void Surface::DisplayBumpImg(string filename_bump){

	// display bump Image
//	cv::Mat BumpImg = cv::Mat::zeros(_m,_n, CV_64F);
	cv::Mat BumpImg = cv::Mat::zeros(_m,_n, CV_8UC3);
//	cv::Mat BumpImg(_m,_n, CV_16U);
//	double maxibump = -1000;

	double val;

	for(int i = 0 ; i<int(_n) ; i++){

		for(int j = 0 ; j<int(_m) ; j++){


			if( _BumpImg[i][j] != -1){
					//if(_BumpImg[i][j] > maxibump){
					//	maxibump = _BumpImg[i][j];
					//}
				val = 6000*abs(_BumpImg[i][j]);// *6000 to adapt to an ushort value
				if(val>255){val = 255;}

				BumpImg.at<cv::Vec3b>(_m-1-j,i)[0] = (unsigned char) (val);
				BumpImg.at<cv::Vec3b>(_m-1-j,i)[1] = (unsigned char) (val);
				BumpImg.at<cv::Vec3b>(_m-1-j,i)[2] = (unsigned char) (val);
			}

			else{
				BumpImg.at<cv::Vec3b>(_m-1-j,i)[0] = 0; 
				BumpImg.at<cv::Vec3b>(_m-1-j,i)[1] = 0;
				BumpImg.at<cv::Vec3b>(_m-1-j,i)[2] = 0;
			}

		}

	}

	//cout<<"maxi bump : "<<maxibump<<endl;

	cv::imshow(filename_bump, BumpImg);
	
	cv::imwrite(filename_bump, BumpImg);
}

void Surface::DisplaySurf(){
//	glPointSize(2);
	glBegin(GL_POINTS);
	glColor4ub(0,255,0, 255);
	for (int i=0; i < _n; i++) {
        for (int j=0; j < _m; j++) { 
			if (_VtxImg[i][j] != NULL) {
				glVertex3f(_VtxImg[i][j][0]/*-2*/, _VtxImg[i][j][1], _VtxImg[i][j][2]);
			}
		}
	}
	glEnd( );
//	glPointSize(1);
}


void Surface::DisplayRecImg(int color){
	glBegin(GL_POINTS);
	if(color == 0){
		glColor4ub(255,0,0, 255);}
	if(color == 1){
		glColor4ub(0,255,0, 255);}
	for (int i=0; i < _n; i++) {
        for (int j=0; j < _m; j++) { 
			if (_RecImg[i][j] != NULL) {
				glVertex3d(_RecImg[i][j][0], _RecImg[i][j][1], _RecImg[i][j][2]);
			}
		}
	}
	glEnd( );
}




void Surface::DisplaySurfsPlus(int switch1){ //switch1 = 0 doesnt display 2D triangles and 2D optimized triangles


	if(switch1){

		glPointSize(4);
		glEnable(GL_POINT_SMOOTH);
		glBegin(GL_POINTS);


		// Opti ControlsIdx  display :
		for(int i = 0 ; i < _OptControlsIdx.size() ; i++){

			if (i==(_OptControlsIdx.size()-1)) {glColor4ub(255,0,255, 255);}
			else {glColor4ub(255,0,0, 255);}

			glVertex3d(_OptControlsIdx[i][0]/double(_k),_OptControlsIdx[i][1]/double(_k),0.0);

		 }


		// ControlsIdx  display :
		for(int i = 0 ; i < _ControlsIdx.size() ; i++){

			if (i==(_ControlsIdx.size()-1)) {glColor4ub(255,150,150, 255);}
			else {glColor4ub(150,150,150, 255);}

			glVertex3d(_ControlsIdx[i][0]/double(_k)-1,_ControlsIdx[i][1]/double(_k)+0.1,0.0);

		 }

		glEnd();

		glDisable(GL_POINT_SMOOTH);

	}

	glBegin(GL_TRIANGLES);

	for(int i=0; i < _Equations.size() ; i++)
	{

		int idx[3];
		int idx3d[3];

		idx[0] = _Equations[i]._idxPos[0];/*_EdgesMeasures[3*i].to;*/
		idx[1] = _Equations[i]._idxPos[1];/*_EdgesMeasures[3*i].from;*/
		idx[2] = _Equations[i]._idxPos[2];/*_EdgesMeasures[3*i].last;*/

		glColor4ub(500*((i+2)*(i+1)),200*(i+1)*(i+5),300*(i+1), 80);
	//	glColor4ub(0, 255, 0, 50);

		// Controls Triangles display :
		glVertex3d(_Controls[idx[0]].x(), _Controls[idx[0]].y(), _Controls[idx[0]].z());
		glVertex3d(_Controls[idx[2]].x(), _Controls[idx[2]].y(), _Controls[idx[2]].z());
		glVertex3d(_Controls[idx[1]].x(), _Controls[idx[1]].y(), _Controls[idx[1]].z());

		glVertex3d(_Controls[idx[0]].x(), _Controls[idx[0]].y(), _Controls[idx[0]].z());// Juste pour dedoubler
		glVertex3d(_Controls[idx[1]].x(), _Controls[idx[1]].y(), _Controls[idx[1]].z());
		glVertex3d(_Controls[idx[2]].x(), _Controls[idx[2]].y(), _Controls[idx[2]].z());

		if(switch1){
			// Opti ControlsIdx Triangles display :
			glVertex3d(_OptControlsIdx[idx[0]][0]/double(_k), _OptControlsIdx[idx[0]][1]/double(_k),0.0);
			glVertex3d(_OptControlsIdx[idx[1]][0]/double(_k), _OptControlsIdx[idx[1]][1]/double(_k),0.0);
			glVertex3d(_OptControlsIdx[idx[2]][0]/double(_k), _OptControlsIdx[idx[2]][1]/double(_k),0.0);


			// ControlsIdx Triangles display :
			glVertex3d(_ControlsIdx[idx[0]].x()/double(_k)-1, _ControlsIdx[idx[0]].y()/double(_k)+0.1,0.0);
			glVertex3d(_ControlsIdx[idx[1]].x()/double(_k)-1, _ControlsIdx[idx[1]].y()/double(_k)+0.1,0.0);
			glVertex3d(_ControlsIdx[idx[2]].x()/double(_k)-1, _ControlsIdx[idx[2]].y()/double(_k)+0.1,0.0);
		}

	}

	glEnd();

	glPointSize(1);
	glDisable(GL_POINT_SMOOTH);
}
