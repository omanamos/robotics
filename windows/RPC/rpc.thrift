namespace csharp Communication
namespace cpp communication

struct Point {
  1: i32 x,
  2: i32 y,
  3: i32 z,
}

struct PointCloud {
  1: list<Point> points,
}

service Rpc {
   void ping(),
   list<PointCloud> getObjects()   
}
