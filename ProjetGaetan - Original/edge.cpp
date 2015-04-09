// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "edge.h"

namespace g2o {
  namespace tutorial {

    Edge::Edge() :
    BaseBinaryEdge<2, MeasType, Vertex, Vertex>()
	//BaseMultiEdge<2, Eigen::Vector2d>()
    {
		/*_information(0,0)=1.0;
		_information(0,1)=0.0;
		_information(1,0)=0.0;
		_information(1,1)=1.0;*/

    }

	void Edge::computeError()
	{		
		const Vertex* v1 = static_cast<const Vertex*>(_vertices[0]);
		const Vertex* v2 = static_cast<const Vertex*>(_vertices[1]);
		//	const Vertex* v3 = static_cast<const Vertex*>(_vertices[2]);
	 	const Vertex* v3 = static_cast<const Vertex*>(_vertex3);

		double V0[2], V1[2], V2[2];
		/*double *V1toBtemp = new double[2];*/

		double V1toBtemp[2];

		V0[0] = v1->estimate().x() ; V0[1] = v1->estimate().y() ;
		V1[0] = v2->estimate().x() ; V1[1] = v2->estimate().y() ;
		V2[0] = v3->estimate().x() ; V2[1] = v3->estimate().y() ;

		base_2d(V0, V1, V2, V1toBtemp);

		_error[0] = 0.5* (sqrt((V1[0]-V0[0])*(V1[0]-V0[0])+(V1[1]-V0[1])*(V1[1]-V0[1])) - _measurement[0]); // (0.5 because this dist will appear in 2 triangles)
		_error[1] = 1 * (sqrt(V1toBtemp[0]*V1toBtemp[0]+V1toBtemp[1]*V1toBtemp[1]) - _measurement[1]);

	//	std::cout << "error :" << _error.transpose() << std::endl;

	}


    bool Edge::read(std::istream& is)
    {

/*
	  Eigen::Vector2d p;
	  is >> p[0] >> p[1] ;
	  _measurement = p;*/


      return true;
    }

    bool Edge::write(std::ostream& os) const
    {
    
	MeasType p = measurement();
	Eigen::Matrix2d inf = information();

	os << p.x() << " " << p.y() << " " << inf(0,0) << " " << inf(0,1) << " " << inf(1,0) << " " << inf(1,1) ;

      return os.good();
    }





  } // end namespace
} // end namespace
