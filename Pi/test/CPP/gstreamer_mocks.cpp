#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gtest/gtest.h>
#include <string>



int gst_init_called = 0;
int pipeline_new_called = 0;
int factory_make_called = 0;
int caps_from_string_called = 0;
int caps_unref_called = 0;
int element_link_called = 0;
int g_object_set_called = 0;
int set_state_called = 0;
int unref_called = 0;
int try_pull_sample_called = 0;
int get_buffer_called = 0;
int get_caps_called = 0;
int get_structure_called = 0;
int structure_get_int_called = 0;
int buffer_map_called = 0;
int buffer_unmap_called = 0;
int sample_unref_called = 0;

GstElement* fake_pipeline = reinterpret_cast<GstElement*>(0x101);
GstElement* fake_source   = reinterpret_cast<GstElement*>(0x102);
GstElement* fake_caps     = reinterpret_cast<GstElement*>(0x103);
GstElement* fake_sink     = reinterpret_cast<GstElement*>(0x104);
GstElement* fake_element     = reinterpret_cast<GstElement*>(0x105);
GstCaps* fake_caps_caps       = reinterpret_cast<GstCaps*>(0x303);
GstSample* fake_sample = reinterpret_cast<GstSample*>(0x404);
GstBuffer* fake_buffer = reinterpret_cast<GstBuffer*>(0x505);
GstStructure* fake_structure = reinterpret_cast<GstStructure*>(0x606);


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
