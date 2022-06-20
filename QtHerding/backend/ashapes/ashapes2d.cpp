#include "ashapes2d.h"

#include <utilities/log.h>
#include <topology/filtration.h>
#include <topology/static-persistence.h>
#include <topology/dynamic-persistence.h>
#include <topology/persistence-diagram.h>
#include <iostream>
#include <time.h>
#include <fstream>


#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>

namespace bg = boost::geometry;


typedef Filtration<ASimplex2D>              AFiltration;
typedef         DynamicPersistenceChains<>      Persistence;
typedef PersistenceDiagram<>                    PDgm;
typedef         ASimplex2D                 Smplx;


typedef bg::model::point<double, 2, bg::cs::cartesian> point_t;
typedef bg::model::polygon<point_t> polygon_t; 

#include <utilities/log.h>
#include <boost/foreach.hpp>

ASimplex2D::
ASimplex2D(const Delaunay2D::Vertex& v): a_(0), attached_(false)
{
    for (int i = 0; i < 3; ++i)
        if (v.face()->vertex(i) != Vertex_handle() && v.face()->vertex(i)->point() == v.point())
            Parent::add(v.face()->vertex(i));
}

ASimplex2D::
ASimplex2D(const Delaunay2D::Edge& e): attached_(false)
{
    Face_handle f = e.first;
    for (int i = 0; i < 3; ++i)
        if (i != e.second)
            Parent::add(f->vertex(i));
}

ASimplex2D::
ASimplex2D(const Delaunay2D::Edge& e, const SimplexSet& simplices, const Delaunay2D& Dt): attached_(false)
{
    Face_handle f = e.first;
    for (int i = 0; i < 3; ++i)
        if (i != e.second)
            Parent::add(f->vertex(i));

    VertexSet::const_iterator v = static_cast<const Parent*>(this)->vertices().begin();
    const Point& p1 = (*v++)->point();
    const Point& p2 = (*v)->point();

    Face_handle o = f->neighbor(e.second);
    if (o == Face_handle())
    {
        a_ = CGAL::squared_radius(p1, p2);
        return;
    }
    int oi = o->index(f);

    attached_ = false;
    if (!Dt.is_infinite(f->vertex(e.second)) &&
        CGAL::side_of_bounded_circle(p1, p2,
                                     f->vertex(e.second)->point()) == CGAL::ON_BOUNDED_SIDE)
        attached_ = true;
    else if (!Dt.is_infinite(o->vertex(oi)) &&
             CGAL::side_of_bounded_circle(p1, p2,
                                          o->vertex(oi)->point()) == CGAL::ON_BOUNDED_SIDE)
        attached_ = true;
    else
        a_ = CGAL::squared_radius(p1, p2);

    if (attached_)
    {
        if (Dt.is_infinite(f))
            a_ = simplices.find(ASimplex2D(*o))->a();
        else if (Dt.is_infinite(o))
            a_ = simplices.find(ASimplex2D(*f))->a();
        else
            a_ = std::min(simplices.find(ASimplex2D(*f))->a(),
                              simplices.find(ASimplex2D(*o))->a());
    }
}

ASimplex2D::
ASimplex2D(const Delaunay2D::Face& f): attached_(false)
{
    for (int i = 0; i < 3; ++i)
        Parent::add(f.vertex(i));
    VertexSet::const_iterator v = static_cast<const Parent*>(this)->vertices().begin();
    Point p1 = (*v++)->point();
    Point p2 = (*v++)->point();
    Point p3 = (*v)->point();
    a_ = CGAL::squared_radius(p1, p2, p3);
}


bool
ASimplex2D::AOrder::
operator()(const ASimplex2D& first, const ASimplex2D& second) const
{
    if (first.a() == second.a())
        return (first.dimension() < second.dimension());
    else
        return (first.a() < second.a());
}

std::ostream&
ASimplex2D::
operator<<(std::ostream& out) const
{
    for (VertexSet::const_iterator cur = Parent::vertices().begin();
                                   cur != Parent::vertices().end(); ++cur)
        out << **cur << ", ";
    out << "value = " << value();

    return out;
}

void fill_simplex_set(const Delaunay2D& Dt, ASimplex2D::SimplexSet& simplices)
{
    for(Face_iterator cur = Dt.finite_faces_begin(); cur != Dt.finite_faces_end(); ++cur)
        simplices.insert(ASimplex2D(*cur));
    rInfo("Faces inserted");
    for(Edge_iterator cur = Dt.finite_edges_begin(); cur != Dt.finite_edges_end(); ++cur)
        simplices.insert(ASimplex2D(*cur, simplices, Dt));
    rInfo("Edges inserted");
    for(Vertex_iterator cur = Dt.finite_vertices_begin(); cur != Dt.finite_vertices_end(); ++cur)
        simplices.insert(ASimplex2D(*cur));
    rInfo("Vertices inserted");
}

template<class Filtration>
void fill_complex(const Delaunay2D& Dt, Filtration& filtration)
{
    // Compute all simplices with their a values and attachment information
    // TODO: this can be optimized; the new Filtration can act as a SimplexSet
    ASimplex2D::SimplexSet simplices;
    fill_simplex_set(Dt, simplices);
    BOOST_FOREACH(const ASimplex2D& s, simplices)
        filtration.push_back(s);
}

void runVerification(std::queue<double> in, double &eps, double &maxEdge, int &cycleSize, double threshold)
{
    const clock_t begin_time = clock();

    SetFrequency(GetCounter("filtration/pair"), 10000);
    SetTrigger(GetCounter("filtration/pair"), GetCounter(""));

    // Read in the point set and compute its Delaunay triangulation
    /*
    std::istream& in = std::cin;
    */
    double x,y;
    Delaunay2D Dt;
    //while(in)
    //std::cout << "points " << std::endl;
    while (!in.empty())
    {
        //in >> x >> y;
    	x = in.front();
    	in.pop();
    	y = in.front();
    	in.pop();

      //std::cout << x << " " << y << std::endl;
        //if (!in) break;
        Point p(x,y);
        Dt.insert(p);
    }

    //rInfo("Delaunay triangulation computed");
   
    AFiltration af;
    fill_complex(Dt, af);
    //rInfo("Simplices: %i", af.size());

    // Create the a-shape filtration
    af.sort(ASimplex2D::AOrder());
    //rInfo("Filtration initialized");

    Persistence p(af);
    //rInfo("Persistence initialized");

    p.pair_simplices();
    //rInfo("Simplices paired");

    Persistence::SimplexMap<AFiltration>    m       = p.make_simplex_map(af);
    std::map<Dimension, PDgm>                   dgms;

    init_diagrams(dgms, p.begin(), p.end(), 
                  evaluate_through_map(m, ASimplex2D::AValueEvaluator()),
                  evaluate_through_map(m, ASimplex2D::DimensionExtractor()));

#if 1
    //std::cout << 0 << std::endl << dgms[0] << std::endl;
    //std::cout << 1 << std::endl << dgms[1] << std::endl;
#endif
    //std::cout << float( clock () - begin_time ) /  CLOCKS_PER_SEC << std::endl;
    //std::cout << "____________________________________" << std::endl;

    // Output cycles
    
    double result = 0.0;
    maxEdge = 0.0;
    for (Persistence::iterator cur = p.begin(); cur != p.end(); ++cur)
    {
    	const Persistence::Cycle& cycle = cur->cycle;
	//std::cout << "cycle " << cycle.size() << std::endl;

    	if (!cur->sign())        // only negative simplices have non-empty cycles
    	{
    		Persistence::OrderIndex birth = cur->pair;      // the cycle that cur killed was born when we added birth (another simplex)
    	    	const Smplx& b = m[birth];
    	    	const Smplx& d = m[cur];
                 
    	    	if (d.dimension() <= 1) continue;

    	    	if (d.value() - b.value() < threshold)
    	    		continue;
    	    	else
              if (result < (d.value() - b.value()))
                result = (d.value() - b.value());

        cycleSize = cycle.size();
        // Iterate over the cycle
        for (Persistence::Cycle::const_iterator si =  cycle.begin(); si != cycle.end(); ++si)
    		{
    	    		const Smplx& s = m[*si];

              const Smplx::VertexContainer vertices = s.vertices();
              auto p1 = vertices[0]->point();
              auto p2 = vertices[1]->point();
              auto x1 = p1[0].exact().to_double();
              double y1 = p1[1].exact().to_double();
              double x2 = p2[0].exact().to_double();
              double y2 = p2[1].exact().to_double();
              maxEdge = fmax(maxEdge, (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
              //std::cout << x1 << " " << y1 << " " << x2 << " " << y2 << std::endl;
    		}  
    	} 	
    }
    eps = result/2;

    //return result;
/*
  // Convert to points
  std::vector<Pnt> points;
  for (int i=0;i<10;++i)
    points.push_back(Point(2*i,2*i+1));

  // Create a polygon
  Polygon polygon;
  polygon.set(points.begin(),points.end());

    
    Pnt int_point(4, 1);

    //std::cout << "within: " << (boost::geometry::within(int_point, polygon) ? "yes" : "no") << std::endl;

    polygon_t poly1;
    bg::append(poly1.outer(), point_t(-1.0, -1.0)); 
    bg::append(poly1.outer(), point_t(-1.0, 5.0));
    bg::append(poly1.outer(), point_t(5.0, 5.0));
    bg::append(poly1.outer(), point_t(5.0, 0.0));
    

    poly1.inners().resize(1); 
    bg::append(poly1.inners()[0], point_t(1.0, 1.0)); 
    bg::append(poly1.inners()[0], point_t(4.0, 1.0));
    bg::append(poly1.inners()[0], point_t(4.0, 4.0));
    bg::append(poly1.inners()[0], point_t(1.0, 4.0));
    bg::append(poly1.inners()[0], point_t(1.0, 1.0));

    point_t int_point(0, 0);

    std::cout << "within: " << (boost::geometry::within(int_point, poly1) ? "yes" : "no") << std::endl;*/
    //return 0;
}

