#ifndef _GFXCONF_H
#define _GFXCONF_H


///////////////////////////////////////////////////////////////////////////
// GDISP                                                                 //
///////////////////////////////////////////////////////////////////////////
#define GFX_USE_GDISP          TRUE
#define GDISP_NEED_VALIDATION  TRUE
#define GDISP_NEED_CLIP        TRUE
// #define GDISP_NEED_CIRCLE                            FALSE
// #define GDISP_NEED_DUALCIRCLE                        FALSE
// #define GDISP_NEED_ELLIPSE                           FALSE
// #define GDISP_NEED_ARC                               FALSE
// #define GDISP_NEED_ARCSECTORS                        FALSE
// #define GDISP_NEED_CONVEX_POLYGON                    FALSE
// #define GDISP_NEED_SCROLL                            FALSE
// #define GDISP_NEED_PIXELREAD                         FALSE
#define GDISP_NEED_CONTROL     TRUE
// #define GDISP_NEED_QUERY                             FALSE
#define GDISP_NEED_MULTITHREAD TRUE
// #define GDISP_NEED_STREAMING                         FALSE
#define GDISP_NEED_TEXT        TRUE
//    #define GDISP_NEED_TEXT_WORDWRAP                 FALSE
//    #define GDISP_NEED_ANTIALIAS                     FALSE
//    #define GDISP_NEED_UTF8                          FALSE
//    #define GDISP_NEED_TEXT_KERNING                  FALSE
//    #define GDISP_INCLUDE_FONT_UI1                   FALSE
//    #define GDISP_INCLUDE_FONT_UI2                   TRUE		// The smallest preferred font.
//    #define GDISP_INCLUDE_FONT_LARGENUMBERS          FALSE
//    #define GDISP_INCLUDE_FONT_DEJAVUSANS10          FALSE
#define GDISP_INCLUDE_FONT_DEJAVUSANS12          TRUE
//    #define GDISP_INCLUDE_FONT_DEJAVUSANS16          FALSE
//    #define GDISP_INCLUDE_FONT_DEJAVUSANS20          FALSE
//    #define GDISP_INCLUDE_FONT_DEJAVUSANS24          TRUE
    #define GDISP_INCLUDE_FONT_DEJAVUSANS32 TRUE
//    #define GDISP_INCLUDE_FONT_DEJAVUSANSBOLD12      FALSE
//    #define GDISP_INCLUDE_FONT_FIXED_10X20           FALSE
//    #define GDISP_INCLUDE_FONT_FIXED_7X14            FALSE
//    #define GDISP_INCLUDE_FONT_FIXED_5X8             FALSE
//    #define GDISP_INCLUDE_FONT_DEJAVUSANS12_AA       FALSE
//    #define GDISP_INCLUDE_FONT_DEJAVUSANS16_AA       FALSE
//    #define GDISP_INCLUDE_FONT_DEJAVUSANS20_AA       FALSE
//    #define GDISP_INCLUDE_FONT_DEJAVUSANS24_AA       FALSE
//    #define GDISP_INCLUDE_FONT_DEJAVUSANS32_AA       FALSE
//    #define GDISP_INCLUDE_FONT_DEJAVUSANSBOLD12_AA   FALSE
//    #define GDISP_INCLUDE_USER_FONTS                 FALSE

// #define GDISP_NEED_PIXMAP                            FALSE
//    #define GDISP_NEED_PIXMAP_IMAGE                  FALSE

#define GDISP_DEFAULT_ORIENTATION GDISP_ROTATE_270
// #define GDISP_LINEBUF_SIZE                           128
// #define GDISP_STARTUP_COLOR                          Black
#define GDISP_NEED_STARTUP_LOGO                      FALSE

///////////////////////////////////////////////////////////////////////////
// GWIN                                                                  //
///////////////////////////////////////////////////////////////////////////
#define GFX_USE_GWIN              TRUE

 #define GWIN_NEED_WINDOWMANAGER  TRUE
 #define GWIN_NEED_CONSOLE                            TRUE
//    #define GWIN_CONSOLE_USE_HISTORY                 FALSE
//        #define GWIN_CONSOLE_HISTORY_AVERAGING       FALSE
//        #define GWIN_CONSOLE_HISTORY_ATCREATE        FALSE
#define GWIN_CONSOLE_ESCSEQ                      TRUE
//    #define GWIN_CONSOLE_USE_BASESTREAM              FALSE
//    #define GWIN_CONSOLE_USE_FLOAT                   FALSE
// #define GWIN_NEED_GRAPH                              FALSE
// #define GWIN_NEED_GL3D                               FALSE

#define GWIN_NEED_WIDGET    TRUE
// #define GWIN_FOCUS_HIGHLIGHT_WIDTH                   1
    #define GWIN_NEED_LABEL TRUE
    #define GWIN_NEED_BUTTON                         TRUE
//        #define GWIN_BUTTON_LAZY_RELEASE             FALSE
#define GWIN_NEED_SLIDER                         TRUE
#define GWIN_SLIDER_NOSNAP                   TRUE
//        #define GWIN_SLIDER_DEAD_BAND                5
//        #define GWIN_SLIDER_TOGGLE_INC               20
#define GWIN_NEED_CHECKBOX                       TRUE
//    #define GWIN_NEED_RADIO                          FALSE
//    #define GWIN_NEED_PROGRESSBAR                    FALSE
//        #define GWIN_PROGRESSBAR_AUTO                FALSE
//    #define GWIN_FLAT_STYLING TRUE
//    #define GWIN_WIDGET_TAGS                         FALSE

// #define GWIN_NEED_CONTAINERS                         FALSE
//    #define GWIN_NEED_CONTAINER                      FALSE
//    #define GWIN_NEED_FRAME                          FALSE
//    #define GWIN_NEED_TABSET                         FALSE
//        #define GWIN_TABSET_TABHEIGHT                18


///////////////////////////////////////////////////////////////////////////
// GEVENT                                                                //
///////////////////////////////////////////////////////////////////////////
#define GFX_USE_GEVENT              TRUE

// #define GEVENT_ASSERT_NO_RESOURCE                    FALSE
// #define GEVENT_MAXIMUM_SIZE                          32
// #define GEVENT_MAX_SOURCE_LISTENERS                  32


///////////////////////////////////////////////////////////////////////////
// GTIMER                                                                //
///////////////////////////////////////////////////////////////////////////
#define GFX_USE_GTIMER              TRUE

#define GTIMER_THREAD_PRIORITY      NORMALPRIO
#define GTIMER_THREAD_WORKAREA_SIZE 2048


///////////////////////////////////////////////////////////////////////////
// GQUEUE                                                                //
///////////////////////////////////////////////////////////////////////////
#define GFX_USE_GQUEUE              TRUE

#define GQUEUE_NEED_ASYNC           TRUE
// #define GQUEUE_NEED_GSYNC                            FALSE
// #define GQUEUE_NEED_FSYNC                            FALSE
// #define GQUEUE_NEED_BUFFERS                          FALSE

///////////////////////////////////////////////////////////////////////////
// GINPUT                                                                //
///////////////////////////////////////////////////////////////////////////
#define GFX_USE_GINPUT              TRUE

#define GINPUT_NEED_MOUSE           TRUE
#define GINPUT_TOUCH_STARTRAW                    FALSE
//    #define GINPUT_TOUCH_NOTOUCH                     FALSE
//#define GINPUT_TOUCH_NOCALIBRATE                 TRUE
//    #define GINPUT_TOUCH_NOCALIBRATE_GUI             FALSE
//    #define GINPUT_MOUSE_POLL_PERIOD                 25
//    #define GINPUT_MOUSE_CLICK_TIME                  300
//    #define GINPUT_TOUCH_CXTCLICK_TIME               700
#define GINPUT_TOUCH_USER_CALIBRATION_LOAD       FALSE
#define GINPUT_TOUCH_USER_CALIBRATION_SAVE       FALSE
//    #define GMOUSE_DRIVER_LIST                       GMOUSEVMT_Win32, GMOUSEVMT_Win32
// #define GINPUT_NEED_TOGGLE                           FALSE
// #define GINPUT_NEED_DIAL                             FALSE

///////////////////////////////////////////////////////////////////////////
// GADC                                                                  //
///////////////////////////////////////////////////////////////////////////
// #define GFX_USE_GADC                                 FALSE
//    #define GADC_MAX_LOWSPEED_DEVICES                4
#define GOS_NEED_X_THREADS FALSE
#define GOX_NEED_X_HEAP    FALSE
#endif /* _GFXCONF_H */
