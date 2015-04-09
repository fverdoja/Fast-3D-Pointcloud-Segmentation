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

#ifndef G2O_TUTORIAL_EDGE_H
#define G2O_TUTORIAL_EDGE_H

#define MeasType Eigen::Vector2d

#include "vertex.h"
#include <iostream>
#include "g2o_tutorial_slam2d_api.h"
#include "g2o/core/base_binary_edge.h"
//#include "g2o/core/base_multi_edge.h"

#include "Surface.h"

namespace g2o {

  namespace tutorial {


 //     typedef std::vector<Vertex*>                      VertexContainer;

    /**
     * \brief 2D edge between two Vertex2, i.e., the odometry
     */
    class G2O_TUTORIAL_SLAM2D_API Edge : public BaseBinaryEdge<2, MeasType, Vertex, Vertex>
    {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        Edge();

        void computeError();

       // void setMeasurement(const double* m){
		void setMeasurement(const MeasType& m){
			
		  _measurement[0] = m[0];
		  _measurement[1] = m[1];
    //      _measurement[2] = m[2];
		  //_measurement[3] = m[3];
        }

		bool setMeasurementFromState(){

		//const Vertex* v1 = static_cast<const Vertex*>(_vertices[0]);
		//const Vertex* v2 = static_cast<const Vertex*>(_vertices[1]);

		//_measurement =(v2->estimate()-v1->estimate());

		return true;
		}

        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;
//        g2o::OptimizableGraph::Vertex* vertex3() { return _vertex3;}
		void setVertex3( g2o::OptimizableGraph::Vertex* v ) 
		{
			_vertex3 = v;
		}

      protected:

		 g2o::OptimizableGraph::Vertex* _vertex3;

    };

  } // end namespace

} // end namespace

#endif
