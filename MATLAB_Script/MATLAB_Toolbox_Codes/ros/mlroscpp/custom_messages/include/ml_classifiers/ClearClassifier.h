// Generated by gencpp from file ml_classifiers/ClearClassifier.msg
// DO NOT EDIT!


#ifndef ML_CLASSIFIERS_MESSAGE_CLEARCLASSIFIER_H
#define ML_CLASSIFIERS_MESSAGE_CLEARCLASSIFIER_H

#include <ros/service_traits.h>


#include <ml_classifiers/ClearClassifierRequest.h>
#include <ml_classifiers/ClearClassifierResponse.h>


namespace ml_classifiers
{

struct ClearClassifier
{

typedef ClearClassifierRequest Request;
typedef ClearClassifierResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ClearClassifier
} // namespace ml_classifiers


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::ml_classifiers::ClearClassifier > {
  static const char* value()
  {
    return "126e80d7c8d36adbb3fd063504a07c0d";
  }

  static const char* value(const ::ml_classifiers::ClearClassifier&) { return value(); }
};

template<>
struct DataType< ::ml_classifiers::ClearClassifier > {
  static const char* value()
  {
    return "ml_classifiers/ClearClassifier";
  }

  static const char* value(const ::ml_classifiers::ClearClassifier&) { return value(); }
};


// service_traits::MD5Sum< ::ml_classifiers::ClearClassifierRequest> should match 
// service_traits::MD5Sum< ::ml_classifiers::ClearClassifier > 
template<>
struct MD5Sum< ::ml_classifiers::ClearClassifierRequest>
{
  static const char* value()
  {
    return MD5Sum< ::ml_classifiers::ClearClassifier >::value();
  }
  static const char* value(const ::ml_classifiers::ClearClassifierRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::ml_classifiers::ClearClassifierRequest> should match 
// service_traits::DataType< ::ml_classifiers::ClearClassifier > 
template<>
struct DataType< ::ml_classifiers::ClearClassifierRequest>
{
  static const char* value()
  {
    return DataType< ::ml_classifiers::ClearClassifier >::value();
  }
  static const char* value(const ::ml_classifiers::ClearClassifierRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::ml_classifiers::ClearClassifierResponse> should match 
// service_traits::MD5Sum< ::ml_classifiers::ClearClassifier > 
template<>
struct MD5Sum< ::ml_classifiers::ClearClassifierResponse>
{
  static const char* value()
  {
    return MD5Sum< ::ml_classifiers::ClearClassifier >::value();
  }
  static const char* value(const ::ml_classifiers::ClearClassifierResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::ml_classifiers::ClearClassifierResponse> should match 
// service_traits::DataType< ::ml_classifiers::ClearClassifier > 
template<>
struct DataType< ::ml_classifiers::ClearClassifierResponse>
{
  static const char* value()
  {
    return DataType< ::ml_classifiers::ClearClassifier >::value();
  }
  static const char* value(const ::ml_classifiers::ClearClassifierResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ML_CLASSIFIERS_MESSAGE_CLEARCLASSIFIER_H