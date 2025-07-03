#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gtest/gtest.h>
#include <string>


int set_state_called = 0;
int unref_called = 0;


extern "C" {

GstStateChangeReturn gst_element_set_state(GstElement* element, GstState state) {
    set_state_called++;
    EXPECT_TRUE(element != nullptr);
    return GST_STATE_CHANGE_SUCCESS;
}

void gst_object_unref(gpointer obj) {
    unref_called++;
    EXPECT_TRUE(obj != nullptr);
}


} // extern "C"
