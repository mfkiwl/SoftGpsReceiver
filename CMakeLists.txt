cmake_minimum_required(VERSION 3.21)

project(SoftGpsReceiver VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets Concurrent)
qt_standard_project_setup()

# Sources are UTF-8 (degree signs etc.)
if(MSVC)
    add_compile_options(/utf-8)
endif()

# The signal-processing core processes ~10^9 samples per channel. Without
# optimisation a Debug build is unusably slow, so by default the core is
# optimised even in Debug builds (the GUI stays fully debuggable).
option(GNSS_OPTIMIZE_CORE_IN_DEBUG "Optimise the DSP core even in Debug builds" ON)

# ---------------------------------------------------------------------------
# Core library: everything that does not need a GUI
# ---------------------------------------------------------------------------
add_library(gnsscore STATIC
    src/core/Constants.h
    src/core/MathUtil.h
    src/core/Settings.h            src/core/Settings.cpp
    src/core/SignalSource.h        src/core/SignalSource.cpp
    src/core/Fft.h                 src/core/Fft.cpp
    src/core/CaCode.h              src/core/CaCode.cpp
    src/core/GalileoCodes.h        src/core/GalileoCodes.cpp
    src/core/GlonassCode.h         src/core/GlonassCode.cpp
    src/core/GlonassNav.h          src/core/GlonassNav.cpp
    src/core/GalileoNav.h          src/core/GalileoNav.cpp
    src/core/SignalAnalysis.h      src/core/SignalAnalysis.cpp
    src/core/Acquisition.h         src/core/Acquisition.cpp
    src/core/Tracking.h            src/core/Tracking.cpp
    src/core/NavMessage.h          src/core/NavMessage.cpp
    src/core/Ephemeris.h           src/core/Ephemeris.cpp
    src/core/Geodesy.h             src/core/Geodesy.cpp
    src/core/Pvt.h                 src/core/Pvt.cpp
    src/core/Receiver.h            src/core/Receiver.cpp
    src/core/LiveRing.h            src/core/LiveRing.cpp
    src/core/LiveInput.h           src/core/ReplayInput.cpp       src/core/UhdInput.cpp
    src/core/SmInput.cpp
    src/core/BandSplitter.h        src/core/BandSplitter.cpp
    src/core/LiveEngine.h          src/core/LiveEngine.cpp
)
target_include_directories(gnsscore PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(gnsscore PUBLIC Qt6::Core Qt6::Concurrent)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(gnsscore PRIVATE -Wall -Wextra -Wno-unused-parameter)
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        target_compile_options(gnsscore PRIVATE -fcx-limited-range)
    endif()
    if(GNSS_OPTIMIZE_CORE_IN_DEBUG)
        target_compile_options(gnsscore PRIVATE $<$<CONFIG:Debug>:-O2>)
    endif()
    target_compile_options(gnsscore PRIVATE $<$<NOT:$<CONFIG:Debug>>:-O3>)
elseif(MSVC)
    target_compile_options(gnsscore PRIVATE /W3 $<$<NOT:$<CONFIG:Debug>>:/O2>)
endif()

# ---------------------------------------------------------------------------
# GUI application. Every page, the main window and the settings dialog are
# Qt Designer forms (.ui) - open them in Qt Creator to edit the layout. The
# plots/tables are promoted widgets (gui::PlotWidget, gui::DataTable,
# gui::SkyPlotWidget); plot titles and axis labels are dynamic properties
# (title, xLabel, yLabel) that can be edited in Designer.
# ---------------------------------------------------------------------------
option(GNSS_BUILD_GUI "Build the Qt Widgets application" ON)
if(GNSS_BUILD_GUI)
set(GNSS_GUI_SOURCES
    src/gui/MainWindow.h           src/gui/MainWindow.cpp         src/gui/MainWindow.ui
    src/gui/SettingsDialog.h       src/gui/SettingsDialog.cpp     src/gui/SettingsDialog.ui
    src/gui/OverviewPage.h         src/gui/OverviewPage.cpp       src/gui/OverviewPage.ui
    src/gui/RawSignalPage.h        src/gui/RawSignalPage.cpp      src/gui/RawSignalPage.ui
    src/gui/CaCodePage.h           src/gui/CaCodePage.cpp         src/gui/CaCodePage.ui
    src/gui/AcquisitionPage.h      src/gui/AcquisitionPage.cpp    src/gui/AcquisitionPage.ui
    src/gui/TrackingPage.h         src/gui/TrackingPage.cpp       src/gui/TrackingPage.ui
    src/gui/BitSyncPage.h          src/gui/BitSyncPage.cpp        src/gui/BitSyncPage.ui
    src/gui/NavMessagePage.h       src/gui/NavMessagePage.cpp     src/gui/NavMessagePage.ui
    src/gui/EphemerisPage.h        src/gui/EphemerisPage.cpp      src/gui/EphemerisPage.ui
    src/gui/SatellitesPage.h       src/gui/SatellitesPage.cpp     src/gui/SatellitesPage.ui
    src/gui/MeasurementsPage.h     src/gui/MeasurementsPage.cpp   src/gui/MeasurementsPage.ui
    src/gui/PositionPage.h         src/gui/PositionPage.cpp       src/gui/PositionPage.ui
    src/gui/LivePage.h             src/gui/LivePage.cpp           src/gui/LivePage.ui
    src/gui/PlotWidget.h           src/gui/PlotWidget.cpp
    src/gui/SkyPlotWidget.h        src/gui/SkyPlotWidget.cpp
    src/gui/DataTable.h            src/gui/DataTable.cpp
    src/gui/GuiUtil.h              src/gui/GuiUtil.cpp
)
qt_add_executable(SoftGpsReceiver WIN32 MACOSX_BUNDLE src/main.cpp ${GNSS_GUI_SOURCES})
qt_add_resources(SoftGpsReceiver "docs"
    PREFIX "/"
    FILES docs/algorithms.html
)
qt_add_resources(SoftGpsReceiver "icon"
    PREFIX "/"
    FILES resources/SoftGpsReceiver.ico
)
if(WIN32)
    target_sources(SoftGpsReceiver PRIVATE resources/SoftGpsReceiver.rc)    # icon of the .exe in Explorer
endif()
target_include_directories(SoftGpsReceiver PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/gui)
target_link_libraries(SoftGpsReceiver PRIVATE gnsscore Qt6::Widgets Qt6::Gui)

# The Signal Hound driver, found next to the executable. Its licence keeps it out of git, so it is optional here. It is loaded at run time, so nothing links to it.
if(WIN32 AND EXISTS ${CMAKE_SOURCE_DIR}/third_party/signal_hound/windows_x64/sm_api.dll)
    add_custom_target(sm_api_dll
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_SOURCE_DIR}/third_party/signal_hound/windows_x64/sm_api.dll $<TARGET_FILE_DIR:SoftGpsReceiver>
        VERBATIM)
    add_dependencies(SoftGpsReceiver sm_api_dll)
endif()

# Help file, next to the executable. Needs Microsoft HTML Help Workshop.
find_program(GNSS_HHC hhc PATHS "C:/Program Files (x86)/HTML Help Workshop" "C:/Program Files/HTML Help Workshop")
if(WIN32 AND GNSS_HHC)
    file(GLOB GNSS_HELP_FILES CONFIGURE_DEPENDS docs/help/*)
    add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/help/SoftGpsReceiver.chm
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/docs/help ${CMAKE_BINARY_DIR}/help
        COMMAND cmd /c .\\build_chm.cmd "${GNSS_HHC}" SoftGpsReceiver.hhp
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/help
        DEPENDS ${GNSS_HELP_FILES} VERBATIM)
    # Part of building the executable itself, as Qt Creator may build only that target before running
    add_custom_target(help_chm DEPENDS ${CMAKE_BINARY_DIR}/help/SoftGpsReceiver.chm)
    add_dependencies(SoftGpsReceiver help_chm)
    add_custom_command(TARGET SoftGpsReceiver POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/help/SoftGpsReceiver.chm $<TARGET_FILE_DIR:SoftGpsReceiver>
        VERBATIM)
elseif(WIN32)
    message(STATUS "HTML Help Workshop not found: SoftGpsReceiver.chm will not be built")
endif()
endif()
# ---------------------------------------------------------------------------
# Command line front end (batch processing / regression testing)
# ---------------------------------------------------------------------------
qt_add_executable(gnss_cli src/cli/main_cli.cpp)
target_link_libraries(gnss_cli PRIVATE gnsscore)

# ---------------------------------------------------------------------------
# Decoder test against the worked example in the Galileo ICD
# ---------------------------------------------------------------------------
option(GNSS_BUILD_TESTS "Build the self-contained tests" OFF)
if(GNSS_BUILD_TESTS)
    enable_testing()
    qt_add_executable(test_galileo tests/test_galileo.cpp)
    target_link_libraries(test_galileo PRIVATE gnsscore)
    add_test(NAME test_galileo COMMAND test_galileo)
    qt_add_executable(test_glonass tests/test_glonass.cpp)
    target_link_libraries(test_glonass PRIVATE gnsscore)
    add_test(NAME test_glonass COMMAND test_glonass)
    qt_add_executable(gen_glonass tests/gen_glonass.cpp)
    target_link_libraries(gen_glonass PRIVATE gnsscore)

    # Adds simulated Galileo satellites to a GPS recording (see add_galileo.cpp)
    qt_add_executable(add_galileo tests/add_galileo.cpp)
    target_link_libraries(add_galileo PRIVATE gnsscore)

    # Drives the real widgets. Given a recording it also tests closing one (see test_gui.cpp)
    if(GNSS_BUILD_GUI)
        qt_add_executable(test_gui tests/test_gui.cpp ${GNSS_GUI_SOURCES})
        target_include_directories(test_gui PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/gui)
        target_link_libraries(test_gui PRIVATE gnsscore Qt6::Widgets Qt6::Gui)
        if(WIN32)
            target_link_libraries(test_gui PRIVATE psapi)
        endif()
        add_test(NAME test_gui COMMAND test_gui)
        set_tests_properties(test_gui PROPERTIES ENVIRONMENT QT_QPA_PLATFORM=offscreen)
    endif()

    # Stands in for the UHD driver, to test the USRP path without a radio (see fake_uhd.cpp)
    if(WIN32)
        add_library(fake_sm SHARED tests/fake_sm/fake_sm.cpp)
        set_target_properties(fake_sm PROPERTIES OUTPUT_NAME sm_api PREFIX "" RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/fake_sm
                              AUTOMOC OFF AUTOUIC OFF AUTORCC OFF)
        add_library(fake_uhd SHARED tests/fake_uhd/fake_uhd.cpp)
        set_target_properties(fake_uhd PROPERTIES OUTPUT_NAME uhd PREFIX "" RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/fake_uhd
                              AUTOMOC OFF AUTOUIC OFF AUTORCC OFF)
    endif()
endif()
