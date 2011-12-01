namespace csharp Communication
namespace cpp communication

struct Point {
  1: double x,
  2: double y,
  3: double z,
}

struct PointCloud {
  1: list<Point> points,
  2: Point average,
  3: string identifier
}

service Rpc {
   void ping(),
   list<PointCloud> getObjects(),
   Point locateNao(),
   update(string oldIdentifier, string newIdentifier)
}
