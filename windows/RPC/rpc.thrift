namespace csharp Communication
namespace cpp communication

struct Point {
  1: double x,
  2: double y,
  3: double z,
}

struct Color {
  1: i32 r,
  2: i32 g,
  3: i32 b,
}

struct PointCloud {
  1: list<Point> points,
  2: Point average,
  # default identifiers should start with and underscore
  3: string identifier
  4: Color color
}

service Rpc {
   void ping(),
   list<PointCloud> getObjects(),
   Point locateNao(),
   
   # return true if updated, false if you don't know about oldIdentifier
   bool update(1:string oldIdentifier, 2:string newIdentifier)
}
