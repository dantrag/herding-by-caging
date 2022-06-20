/**
 * Author: Dmitriy Morozov
 * Department of Computer Science, Duke University, 2007
 */

#ifndef __ASHAPES2D_H__
#define __ASHAPES2D_H__

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>

#include <topology/simplex.h>
#include <utilities/types.h>

#include <vector>
#include <set>
#include <iostream>
#include <queue>

struct K: CGAL::Exact_predicates_exact_constructions_kernel {};

typedef CGAL::Delaunay_triangulation_2<K>           Delaunay2D;
typedef Delaunay2D::Point                           Point;
typedef Delaunay2D::Vertex_handle                   Vertex_handle;
typedef Delaunay2D::Face_handle                     Face_handle;
typedef K::FT                                       RealValue;

typedef Delaunay2D::Finite_vertices_iterator        Vertex_iterator;
typedef Delaunay2D::Finite_edges_iterator           Edge_iterator;
typedef Delaunay2D::Finite_faces_iterator           Face_iterator;


class ASimplex2D: public Simplex<Vertex_handle>
{
    public:
        typedef     Simplex<Vertex_handle>                              Parent;
        typedef     std::set<ASimplex2D, Parent::VertexComparison>  SimplexSet;
        typedef     Parent::VertexContainer                             VertexSet;

    public:
                                    ASimplex2D()                    {}
                                    ASimplex2D(const Parent& p):
                                            Parent(p)                   {}
                                    ASimplex2D(const ASimplex2D& s):
                                            Parent(s)                   { attached_ = s.attached_; a_ = s.a_; }
                                    ASimplex2D(const Delaunay2D::Vertex& v);
        
                                    ASimplex2D(const Delaunay2D::Edge& e);
                                    ASimplex2D(const Delaunay2D::Edge& e, const SimplexSet& simplices, const Delaunay2D& Dt);
        
                                    ASimplex2D(const Delaunay2D::Face& c);
        
        RealType                    value() const                       { return CGAL::to_double(a_); }
        RealValue                   a() const                       { return a_; }
        bool                        attached() const                    { return attached_; }

        // Ordering
        struct AOrder
        { bool operator()(const ASimplex2D& first, const ASimplex2D& second) const; };
        
        struct AValueEvaluator
        { 
            typedef                 ASimplex2D                                  first_argument_type;
            typedef                 RealType                                        result_type;

            RealType                operator()(const ASimplex2D& s) const       { return s.value(); }
        };

        std::ostream&               operator<<(std::ostream& out) const;
        
    private:
        RealValue                   a_;
        bool                        attached_;
};

typedef             std::vector<ASimplex2D>                             ASimplex2DVector;
void                fill_simplex_set(const Delaunay2D& Dt, ASimplex2D::SimplexSet& simplices);
template<class Filtration>
void                fill_complex(const Delaunay2D& Dt,     Filtration& filtration);

//std::ostream&       operator<<(std::ostream& out, const ASimplex2D& s)  { return s.operator<<(out); }

//#include "ashapes2d.hpp"

void runVerification(std::queue<double> in, double &eps, double &maxEdge, int &cycleSize, double threshold = 0.25);

#endif // __ASHAPES2D_H__
