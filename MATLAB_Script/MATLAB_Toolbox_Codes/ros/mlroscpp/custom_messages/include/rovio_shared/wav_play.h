// Generated by gencpp from file rovio_shared/wav_play.msg
// DO NOT EDIT!


#ifndef ROVIO_SHARED_MESSAGE_WAV_PLAY_H
#define ROVIO_SHARED_MESSAGE_WAV_PLAY_H

#include <ros/service_traits.h>


#include <rovio_shared/wav_playRequest.h>
#include <rovio_shared/wav_playResponse.h>


namespace rovio_shared
{

struct wav_play
{

typedef wav_playRequest Request;
typedef wav_playResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct wav_play
} // namespace rovio_shared


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::rovio_shared::wav_play > {
  static const char* value()
  {
    return "b7ec5ba08b681050147d22f3cf073480";
  }

  static const char* value(const ::rovio_shared::wav_play&) { return value(); }
};

template<>
struct DataType< ::rovio_shared::wav_play > {
  static const char* value()
  {
    return "rovio_shared/wav_play";
  }

  static const char* value(const ::rovio_shared::wav_play&) { return value(); }
};


// service_traits::MD5Sum< ::rovio_shared::wav_playRequest> should match 
// service_traits::MD5Sum< ::rovio_shared::wav_play > 
template<>
struct MD5Sum< ::rovio_shared::wav_playRequest>
{
  static const char* value()
  {
    return MD5Sum< ::rovio_shared::wav_play >::value();
  }
  static const char* value(const ::rovio_shared::wav_playRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::rovio_shared::wav_playRequest> should match 
// service_traits::DataType< ::rovio_shared::wav_play > 
template<>
struct DataType< ::rovio_shared::wav_playRequest>
{
  static const char* value()
  {
    return DataType< ::rovio_shared::wav_play >::value();
  }
  static const char* value(const ::rovio_shared::wav_playRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::rovio_shared::wav_playResponse> should match 
// service_traits::MD5Sum< ::rovio_shared::wav_play > 
template<>
struct MD5Sum< ::rovio_shared::wav_playResponse>
{
  static const char* value()
  {
    return MD5Sum< ::rovio_shared::wav_play >::value();
  }
  static const char* value(const ::rovio_shared::wav_playResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::rovio_shared::wav_playResponse> should match 
// service_traits::DataType< ::rovio_shared::wav_play > 
template<>
struct DataType< ::rovio_shared::wav_playResponse>
{
  static const char* value()
  {
    return DataType< ::rovio_shared::wav_play >::value();
  }
  static const char* value(const ::rovio_shared::wav_playResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ROVIO_SHARED_MESSAGE_WAV_PLAY_H