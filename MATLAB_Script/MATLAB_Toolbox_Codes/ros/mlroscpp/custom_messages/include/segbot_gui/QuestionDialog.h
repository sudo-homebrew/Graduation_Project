// Generated by gencpp from file segbot_gui/QuestionDialog.msg
// DO NOT EDIT!


#ifndef SEGBOT_GUI_MESSAGE_QUESTIONDIALOG_H
#define SEGBOT_GUI_MESSAGE_QUESTIONDIALOG_H

#include <ros/service_traits.h>


#include <segbot_gui/QuestionDialogRequest.h>
#include <segbot_gui/QuestionDialogResponse.h>


namespace segbot_gui
{

struct QuestionDialog
{

typedef QuestionDialogRequest Request;
typedef QuestionDialogResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct QuestionDialog
} // namespace segbot_gui


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::segbot_gui::QuestionDialog > {
  static const char* value()
  {
    return "51da90b68e7df9db553b097d9dbe22a3";
  }

  static const char* value(const ::segbot_gui::QuestionDialog&) { return value(); }
};

template<>
struct DataType< ::segbot_gui::QuestionDialog > {
  static const char* value()
  {
    return "segbot_gui/QuestionDialog";
  }

  static const char* value(const ::segbot_gui::QuestionDialog&) { return value(); }
};


// service_traits::MD5Sum< ::segbot_gui::QuestionDialogRequest> should match 
// service_traits::MD5Sum< ::segbot_gui::QuestionDialog > 
template<>
struct MD5Sum< ::segbot_gui::QuestionDialogRequest>
{
  static const char* value()
  {
    return MD5Sum< ::segbot_gui::QuestionDialog >::value();
  }
  static const char* value(const ::segbot_gui::QuestionDialogRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::segbot_gui::QuestionDialogRequest> should match 
// service_traits::DataType< ::segbot_gui::QuestionDialog > 
template<>
struct DataType< ::segbot_gui::QuestionDialogRequest>
{
  static const char* value()
  {
    return DataType< ::segbot_gui::QuestionDialog >::value();
  }
  static const char* value(const ::segbot_gui::QuestionDialogRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::segbot_gui::QuestionDialogResponse> should match 
// service_traits::MD5Sum< ::segbot_gui::QuestionDialog > 
template<>
struct MD5Sum< ::segbot_gui::QuestionDialogResponse>
{
  static const char* value()
  {
    return MD5Sum< ::segbot_gui::QuestionDialog >::value();
  }
  static const char* value(const ::segbot_gui::QuestionDialogResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::segbot_gui::QuestionDialogResponse> should match 
// service_traits::DataType< ::segbot_gui::QuestionDialog > 
template<>
struct DataType< ::segbot_gui::QuestionDialogResponse>
{
  static const char* value()
  {
    return DataType< ::segbot_gui::QuestionDialog >::value();
  }
  static const char* value(const ::segbot_gui::QuestionDialogResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // SEGBOT_GUI_MESSAGE_QUESTIONDIALOG_H