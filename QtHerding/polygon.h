class Polygon
 {
 	private : 
     		int size;
     		int* len;
 	public:
     		Polygon();
     		Polygon(int k,int *l);
     		~Polygon();
     		int getLength();
     		void setPolygon(int k,int *l);
     		int getEdge(int k);
 };
