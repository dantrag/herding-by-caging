#include "polygon.h" 
Polygon::Polygon()
 {
 	size=0;
 } 
    
Polygon::~Polygon()
 {
 	delete [] len;
 }

Polygon::Polygon(int k,int *l)
 {
 	size=k;
 	len=new int[size];
 	for(int i=0;i<size;i++)len[i]=l[i];
 }

int Polygon::getLength()
 {
 	int sum=0;
 	for(int i=0;i<size;i++) sum+=len[i];
 }

void Polygon::setPolygon(int k,int *l)
 {
 	size=k;
 	len=new int[size];
 	for(int i=0;i<size;i++) len[i]=l[i];
 }

int Polygon::getEdge(int k)
 {
 	return len[k];
 }
