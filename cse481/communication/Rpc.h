/**
 * Autogenerated by Thrift Compiler (0.7.0)
 *
 * DO NOT EDIT UNLESS YOU ARE SURE THAT YOU KNOW WHAT YOU ARE DOING
 */
#ifndef Rpc_H
#define Rpc_H

#include <TProcessor.h>
#include "rpc_types.h"

namespace communication {

class RpcIf {
 public:
  virtual ~RpcIf() {}
  virtual void ping() = 0;
  virtual void getObjects(std::vector<PointCloud> & _return) = 0;
  virtual void locateNao(Point& _return) = 0;
  virtual bool update(const std::string& oldIdentifier, const std::string& newIdentifier) = 0;
};

class RpcNull : virtual public RpcIf {
 public:
  virtual ~RpcNull() {}
  void ping() {
    return;
  }
  void getObjects(std::vector<PointCloud> & /* _return */) {
    return;
  }
  void locateNao(Point& /* _return */) {
    return;
  }
  bool update(const std::string& /* oldIdentifier */, const std::string& /* newIdentifier */) {
    bool _return = false;
    return _return;
  }
};


class Rpc_ping_args {
 public:

  Rpc_ping_args() {
  }

  virtual ~Rpc_ping_args() throw() {}


  bool operator == (const Rpc_ping_args & /* rhs */) const
  {
    return true;
  }
  bool operator != (const Rpc_ping_args &rhs) const {
    return !(*this == rhs);
  }

  bool operator < (const Rpc_ping_args & ) const;

  uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
  uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

};


class Rpc_ping_pargs {
 public:


  virtual ~Rpc_ping_pargs() throw() {}


  uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

};


class Rpc_ping_result {
 public:

  Rpc_ping_result() {
  }

  virtual ~Rpc_ping_result() throw() {}


  bool operator == (const Rpc_ping_result & /* rhs */) const
  {
    return true;
  }
  bool operator != (const Rpc_ping_result &rhs) const {
    return !(*this == rhs);
  }

  bool operator < (const Rpc_ping_result & ) const;

  uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
  uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

};


class Rpc_ping_presult {
 public:


  virtual ~Rpc_ping_presult() throw() {}


  uint32_t read(::apache::thrift::protocol::TProtocol* iprot);

};


class Rpc_getObjects_args {
 public:

  Rpc_getObjects_args() {
  }

  virtual ~Rpc_getObjects_args() throw() {}


  bool operator == (const Rpc_getObjects_args & /* rhs */) const
  {
    return true;
  }
  bool operator != (const Rpc_getObjects_args &rhs) const {
    return !(*this == rhs);
  }

  bool operator < (const Rpc_getObjects_args & ) const;

  uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
  uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

};


class Rpc_getObjects_pargs {
 public:


  virtual ~Rpc_getObjects_pargs() throw() {}


  uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

};

typedef struct _Rpc_getObjects_result__isset {
  _Rpc_getObjects_result__isset() : success(false) {}
  bool success;
} _Rpc_getObjects_result__isset;

class Rpc_getObjects_result {
 public:

  Rpc_getObjects_result() {
  }

  virtual ~Rpc_getObjects_result() throw() {}

  std::vector<PointCloud>  success;

  _Rpc_getObjects_result__isset __isset;

  void __set_success(const std::vector<PointCloud> & val) {
    success = val;
  }

  bool operator == (const Rpc_getObjects_result & rhs) const
  {
    if (!(success == rhs.success))
      return false;
    return true;
  }
  bool operator != (const Rpc_getObjects_result &rhs) const {
    return !(*this == rhs);
  }

  bool operator < (const Rpc_getObjects_result & ) const;

  uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
  uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

};

typedef struct _Rpc_getObjects_presult__isset {
  _Rpc_getObjects_presult__isset() : success(false) {}
  bool success;
} _Rpc_getObjects_presult__isset;

class Rpc_getObjects_presult {
 public:


  virtual ~Rpc_getObjects_presult() throw() {}

  std::vector<PointCloud> * success;

  _Rpc_getObjects_presult__isset __isset;

  uint32_t read(::apache::thrift::protocol::TProtocol* iprot);

};


class Rpc_locateNao_args {
 public:

  Rpc_locateNao_args() {
  }

  virtual ~Rpc_locateNao_args() throw() {}


  bool operator == (const Rpc_locateNao_args & /* rhs */) const
  {
    return true;
  }
  bool operator != (const Rpc_locateNao_args &rhs) const {
    return !(*this == rhs);
  }

  bool operator < (const Rpc_locateNao_args & ) const;

  uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
  uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

};


class Rpc_locateNao_pargs {
 public:


  virtual ~Rpc_locateNao_pargs() throw() {}


  uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

};

typedef struct _Rpc_locateNao_result__isset {
  _Rpc_locateNao_result__isset() : success(false) {}
  bool success;
} _Rpc_locateNao_result__isset;

class Rpc_locateNao_result {
 public:

  Rpc_locateNao_result() {
  }

  virtual ~Rpc_locateNao_result() throw() {}

  Point success;

  _Rpc_locateNao_result__isset __isset;

  void __set_success(const Point& val) {
    success = val;
  }

  bool operator == (const Rpc_locateNao_result & rhs) const
  {
    if (!(success == rhs.success))
      return false;
    return true;
  }
  bool operator != (const Rpc_locateNao_result &rhs) const {
    return !(*this == rhs);
  }

  bool operator < (const Rpc_locateNao_result & ) const;

  uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
  uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

};

typedef struct _Rpc_locateNao_presult__isset {
  _Rpc_locateNao_presult__isset() : success(false) {}
  bool success;
} _Rpc_locateNao_presult__isset;

class Rpc_locateNao_presult {
 public:


  virtual ~Rpc_locateNao_presult() throw() {}

  Point* success;

  _Rpc_locateNao_presult__isset __isset;

  uint32_t read(::apache::thrift::protocol::TProtocol* iprot);

};

typedef struct _Rpc_update_args__isset {
  _Rpc_update_args__isset() : oldIdentifier(false), newIdentifier(false) {}
  bool oldIdentifier;
  bool newIdentifier;
} _Rpc_update_args__isset;

class Rpc_update_args {
 public:

  Rpc_update_args() : oldIdentifier(""), newIdentifier("") {
  }

  virtual ~Rpc_update_args() throw() {}

  std::string oldIdentifier;
  std::string newIdentifier;

  _Rpc_update_args__isset __isset;

  void __set_oldIdentifier(const std::string& val) {
    oldIdentifier = val;
  }

  void __set_newIdentifier(const std::string& val) {
    newIdentifier = val;
  }

  bool operator == (const Rpc_update_args & rhs) const
  {
    if (!(oldIdentifier == rhs.oldIdentifier))
      return false;
    if (!(newIdentifier == rhs.newIdentifier))
      return false;
    return true;
  }
  bool operator != (const Rpc_update_args &rhs) const {
    return !(*this == rhs);
  }

  bool operator < (const Rpc_update_args & ) const;

  uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
  uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

};


class Rpc_update_pargs {
 public:


  virtual ~Rpc_update_pargs() throw() {}

  const std::string* oldIdentifier;
  const std::string* newIdentifier;

  uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

};

typedef struct _Rpc_update_result__isset {
  _Rpc_update_result__isset() : success(false) {}
  bool success;
} _Rpc_update_result__isset;

class Rpc_update_result {
 public:

  Rpc_update_result() : success(0) {
  }

  virtual ~Rpc_update_result() throw() {}

  bool success;

  _Rpc_update_result__isset __isset;

  void __set_success(const bool val) {
    success = val;
  }

  bool operator == (const Rpc_update_result & rhs) const
  {
    if (!(success == rhs.success))
      return false;
    return true;
  }
  bool operator != (const Rpc_update_result &rhs) const {
    return !(*this == rhs);
  }

  bool operator < (const Rpc_update_result & ) const;

  uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
  uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

};

typedef struct _Rpc_update_presult__isset {
  _Rpc_update_presult__isset() : success(false) {}
  bool success;
} _Rpc_update_presult__isset;

class Rpc_update_presult {
 public:


  virtual ~Rpc_update_presult() throw() {}

  bool* success;

  _Rpc_update_presult__isset __isset;

  uint32_t read(::apache::thrift::protocol::TProtocol* iprot);

};

class RpcClient : virtual public RpcIf {
 public:
  RpcClient(boost::shared_ptr< ::apache::thrift::protocol::TProtocol> prot) :
    piprot_(prot),
    poprot_(prot) {
    iprot_ = prot.get();
    oprot_ = prot.get();
  }
  RpcClient(boost::shared_ptr< ::apache::thrift::protocol::TProtocol> iprot, boost::shared_ptr< ::apache::thrift::protocol::TProtocol> oprot) :
    piprot_(iprot),
    poprot_(oprot) {
    iprot_ = iprot.get();
    oprot_ = oprot.get();
  }
  boost::shared_ptr< ::apache::thrift::protocol::TProtocol> getInputProtocol() {
    return piprot_;
  }
  boost::shared_ptr< ::apache::thrift::protocol::TProtocol> getOutputProtocol() {
    return poprot_;
  }
  void ping();
  void send_ping();
  void recv_ping();
  void getObjects(std::vector<PointCloud> & _return);
  void send_getObjects();
  void recv_getObjects(std::vector<PointCloud> & _return);
  void locateNao(Point& _return);
  void send_locateNao();
  void recv_locateNao(Point& _return);
  bool update(const std::string& oldIdentifier, const std::string& newIdentifier);
  void send_update(const std::string& oldIdentifier, const std::string& newIdentifier);
  bool recv_update();
 protected:
  boost::shared_ptr< ::apache::thrift::protocol::TProtocol> piprot_;
  boost::shared_ptr< ::apache::thrift::protocol::TProtocol> poprot_;
  ::apache::thrift::protocol::TProtocol* iprot_;
  ::apache::thrift::protocol::TProtocol* oprot_;
};

class RpcProcessor : virtual public ::apache::thrift::TProcessor {
 protected:
  boost::shared_ptr<RpcIf> iface_;
  virtual bool process_fn(::apache::thrift::protocol::TProtocol* iprot, ::apache::thrift::protocol::TProtocol* oprot, std::string& fname, int32_t seqid, void* callContext);
 private:
  std::map<std::string, void (RpcProcessor::*)(int32_t, ::apache::thrift::protocol::TProtocol*, ::apache::thrift::protocol::TProtocol*, void*)> processMap_;
  void process_ping(int32_t seqid, ::apache::thrift::protocol::TProtocol* iprot, ::apache::thrift::protocol::TProtocol* oprot, void* callContext);
  void process_getObjects(int32_t seqid, ::apache::thrift::protocol::TProtocol* iprot, ::apache::thrift::protocol::TProtocol* oprot, void* callContext);
  void process_locateNao(int32_t seqid, ::apache::thrift::protocol::TProtocol* iprot, ::apache::thrift::protocol::TProtocol* oprot, void* callContext);
  void process_update(int32_t seqid, ::apache::thrift::protocol::TProtocol* iprot, ::apache::thrift::protocol::TProtocol* oprot, void* callContext);
 public:
  RpcProcessor(boost::shared_ptr<RpcIf> iface) :
    iface_(iface) {
    processMap_["ping"] = &RpcProcessor::process_ping;
    processMap_["getObjects"] = &RpcProcessor::process_getObjects;
    processMap_["locateNao"] = &RpcProcessor::process_locateNao;
    processMap_["update"] = &RpcProcessor::process_update;
  }

  virtual bool process(boost::shared_ptr< ::apache::thrift::protocol::TProtocol> piprot, boost::shared_ptr< ::apache::thrift::protocol::TProtocol> poprot, void* callContext);
  virtual ~RpcProcessor() {}
};

class RpcMultiface : virtual public RpcIf {
 public:
  RpcMultiface(std::vector<boost::shared_ptr<RpcIf> >& ifaces) : ifaces_(ifaces) {
  }
  virtual ~RpcMultiface() {}
 protected:
  std::vector<boost::shared_ptr<RpcIf> > ifaces_;
  RpcMultiface() {}
  void add(boost::shared_ptr<RpcIf> iface) {
    ifaces_.push_back(iface);
  }
 public:
  void ping() {
    size_t sz = ifaces_.size();
    for (size_t i = 0; i < sz; ++i) {
      ifaces_[i]->ping();
    }
  }

  void getObjects(std::vector<PointCloud> & _return) {
    size_t sz = ifaces_.size();
    for (size_t i = 0; i < sz; ++i) {
      if (i == sz - 1) {
        ifaces_[i]->getObjects(_return);
        return;
      } else {
        ifaces_[i]->getObjects(_return);
      }
    }
  }

  void locateNao(Point& _return) {
    size_t sz = ifaces_.size();
    for (size_t i = 0; i < sz; ++i) {
      if (i == sz - 1) {
        ifaces_[i]->locateNao(_return);
        return;
      } else {
        ifaces_[i]->locateNao(_return);
      }
    }
  }

  bool update(const std::string& oldIdentifier, const std::string& newIdentifier) {
    size_t sz = ifaces_.size();
    for (size_t i = 0; i < sz; ++i) {
      if (i == sz - 1) {
        return ifaces_[i]->update(oldIdentifier, newIdentifier);
      } else {
        ifaces_[i]->update(oldIdentifier, newIdentifier);
      }
    }
  }

};

} // namespace

#endif
