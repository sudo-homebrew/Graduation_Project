// Generated by gencpp from file mongodb_store/MongoUpdate.msg
// DO NOT EDIT!


#ifndef MONGODB_STORE_MESSAGE_MONGOUPDATE_H
#define MONGODB_STORE_MESSAGE_MONGOUPDATE_H

#include <ros/service_traits.h>


#include <mongodb_store/MongoUpdateRequest.h>
#include <mongodb_store/MongoUpdateResponse.h>


namespace mongodb_store
{

struct MongoUpdate
{

typedef MongoUpdateRequest Request;
typedef MongoUpdateResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct MongoUpdate
} // namespace mongodb_store


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::mongodb_store::MongoUpdate > {
  static const char* value()
  {
    return "2806c067c40cdb2b9e11634221c51298";
  }

  static const char* value(const ::mongodb_store::MongoUpdate&) { return value(); }
};

template<>
struct DataType< ::mongodb_store::MongoUpdate > {
  static const char* value()
  {
    return "mongodb_store/MongoUpdate";
  }

  static const char* value(const ::mongodb_store::MongoUpdate&) { return value(); }
};


// service_traits::MD5Sum< ::mongodb_store::MongoUpdateRequest> should match 
// service_traits::MD5Sum< ::mongodb_store::MongoUpdate > 
template<>
struct MD5Sum< ::mongodb_store::MongoUpdateRequest>
{
  static const char* value()
  {
    return MD5Sum< ::mongodb_store::MongoUpdate >::value();
  }
  static const char* value(const ::mongodb_store::MongoUpdateRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::mongodb_store::MongoUpdateRequest> should match 
// service_traits::DataType< ::mongodb_store::MongoUpdate > 
template<>
struct DataType< ::mongodb_store::MongoUpdateRequest>
{
  static const char* value()
  {
    return DataType< ::mongodb_store::MongoUpdate >::value();
  }
  static const char* value(const ::mongodb_store::MongoUpdateRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::mongodb_store::MongoUpdateResponse> should match 
// service_traits::MD5Sum< ::mongodb_store::MongoUpdate > 
template<>
struct MD5Sum< ::mongodb_store::MongoUpdateResponse>
{
  static const char* value()
  {
    return MD5Sum< ::mongodb_store::MongoUpdate >::value();
  }
  static const char* value(const ::mongodb_store::MongoUpdateResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::mongodb_store::MongoUpdateResponse> should match 
// service_traits::DataType< ::mongodb_store::MongoUpdate > 
template<>
struct DataType< ::mongodb_store::MongoUpdateResponse>
{
  static const char* value()
  {
    return DataType< ::mongodb_store::MongoUpdate >::value();
  }
  static const char* value(const ::mongodb_store::MongoUpdateResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MONGODB_STORE_MESSAGE_MONGOUPDATE_H