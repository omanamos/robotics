/**
 * Autogenerated by Thrift Compiler (0.7.0)
 *
 * DO NOT EDIT UNLESS YOU ARE SURE THAT YOU KNOW WHAT YOU ARE DOING
 */
using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using System.IO;
using Thrift;
using Thrift.Collections;
using Thrift.Protocol;
using Thrift.Transport;
namespace Communication
{

  [Serializable]
  public partial class Point : TBase
  {
    private double _x;
    private double _y;
    private double _z;

    public double X
    {
      get
      {
        return _x;
      }
      set
      {
        __isset.x = true;
        this._x = value;
      }
    }

    public double Y
    {
      get
      {
        return _y;
      }
      set
      {
        __isset.y = true;
        this._y = value;
      }
    }

    public double Z
    {
      get
      {
        return _z;
      }
      set
      {
        __isset.z = true;
        this._z = value;
      }
    }


    public Isset __isset;
    [Serializable]
    public struct Isset {
      public bool x;
      public bool y;
      public bool z;
    }

    public Point() {
    }

    public void Read (TProtocol iprot)
    {
      TField field;
      iprot.ReadStructBegin();
      while (true)
      {
        field = iprot.ReadFieldBegin();
        if (field.Type == TType.Stop) { 
          break;
        }
        switch (field.ID)
        {
          case 1:
            if (field.Type == TType.Double) {
              X = iprot.ReadDouble();
            } else { 
              TProtocolUtil.Skip(iprot, field.Type);
            }
            break;
          case 2:
            if (field.Type == TType.Double) {
              Y = iprot.ReadDouble();
            } else { 
              TProtocolUtil.Skip(iprot, field.Type);
            }
            break;
          case 3:
            if (field.Type == TType.Double) {
              Z = iprot.ReadDouble();
            } else { 
              TProtocolUtil.Skip(iprot, field.Type);
            }
            break;
          default: 
            TProtocolUtil.Skip(iprot, field.Type);
            break;
        }
        iprot.ReadFieldEnd();
      }
      iprot.ReadStructEnd();
    }

    public void Write(TProtocol oprot) {
      TStruct struc = new TStruct("Point");
      oprot.WriteStructBegin(struc);
      TField field = new TField();
      if (__isset.x) {
        field.Name = "x";
        field.Type = TType.Double;
        field.ID = 1;
        oprot.WriteFieldBegin(field);
        oprot.WriteDouble(X);
        oprot.WriteFieldEnd();
      }
      if (__isset.y) {
        field.Name = "y";
        field.Type = TType.Double;
        field.ID = 2;
        oprot.WriteFieldBegin(field);
        oprot.WriteDouble(Y);
        oprot.WriteFieldEnd();
      }
      if (__isset.z) {
        field.Name = "z";
        field.Type = TType.Double;
        field.ID = 3;
        oprot.WriteFieldBegin(field);
        oprot.WriteDouble(Z);
        oprot.WriteFieldEnd();
      }
      oprot.WriteFieldStop();
      oprot.WriteStructEnd();
    }

    public override string ToString() {
      StringBuilder sb = new StringBuilder("Point(");
      sb.Append("X: ");
      sb.Append(X);
      sb.Append(",Y: ");
      sb.Append(Y);
      sb.Append(",Z: ");
      sb.Append(Z);
      sb.Append(")");
      return sb.ToString();
    }

  }

}
