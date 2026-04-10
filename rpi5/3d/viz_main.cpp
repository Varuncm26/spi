/*
 * viz_main.cpp
 * MPU6500 3D OpenCV Visualizer — Raspberry Pi 5
 *
 * Replaces main.c. Compile with:
 *   g++ viz_main.cpp mpu6500.c spi.c -o imu_viz \
 *       $(pkg-config --cflags --libs opencv4) -lm
 *
 * Dependencies:
 *   sudo apt install libopencv-dev
 *
 * The window shows:
 *   • A 3D coloured box that tracks Roll / Pitch in real-time
 *   • XYZ world-frame reference axes
 *   • Live accelerometer, gyroscope and temperature readouts
 *   • Roll / Pitch angle strip at the bottom
 *
 * Press 'q' or ESC to quit.
 */

#include <opencv2/opencv.hpp>
#include <cmath>
#include <cstdio>
#include <unistd.h>
#include <chrono>

extern "C" {
    #include "spi.h"
    #include "mpu6500.h"
}

/* ------------------------------------------------------------------ */
/*  Window / display constants                                          */
/* ------------------------------------------------------------------ */
static constexpr int   WIN_W       = 900;
static constexpr int   WIN_H       = 700;
static constexpr int   CX          = WIN_W / 2;          // canvas centre X
static constexpr int   CY          = WIN_H / 2 - 40;     // canvas centre Y (shifted up)
static constexpr float FOV_SCALE   = 220.0f;             // perspective "focal length"
static constexpr float CUBE_HALF   = 1.0f;               // half-size of the box
static constexpr float CUBE_HX     = CUBE_HALF * 1.8f;   // wider (board shape)
static constexpr float CUBE_HY     = CUBE_HALF * 0.4f;   // thin
static constexpr float CUBE_HZ     = CUBE_HALF * 1.2f;

/* Complementary filter weight — higher = trust gyro more */
static constexpr float CF_ALPHA    = 0.96f;

/* Fixed camera tilt so we see the box from a slight angle */
static constexpr float CAM_ELEV   = 20.0f * M_PI / 180.0f;

/* ------------------------------------------------------------------ */
/*  Simple 3-vector / 3x3 matrix helpers (no Eigen / GLM needed)       */
/* ------------------------------------------------------------------ */
struct Vec3 { float x, y, z; };

static Vec3 rotX(Vec3 v, float a) {
    float c = cosf(a), s = sinf(a);
    return { v.x, c*v.y - s*v.z, s*v.y + c*v.z };
}
static Vec3 rotY(Vec3 v, float a) {
    float c = cosf(a), s = sinf(a);
    return { c*v.x + s*v.z, v.y, -s*v.x + c*v.z };
}
static Vec3 rotZ(Vec3 v, float a) {
    float c = cosf(a), s = sinf(a);
    return { c*v.x - s*v.y, s*v.x + c*v.y, v.z };
}

/* Perspective projection → pixel coordinates */
static cv::Point project(Vec3 v, float camDist = 5.0f) {
    float zp = camDist - v.z;                    // ensure positive depth
    if (zp < 0.1f) zp = 0.1f;
    int px = (int)(CX + FOV_SCALE * v.x / zp);
    int py = (int)(CY - FOV_SCALE * v.y / zp);  // y-axis flipped for screen
    return { px, py };
}

/* ------------------------------------------------------------------ */
/*  Box geometry: 8 vertices, 12 edges, 6 face quads                   */
/* ------------------------------------------------------------------ */
static Vec3 makeVertex(int i) {
    // i encodes a 3-bit mask: bit0=X, bit1=Y, bit2=Z
    return {
        (i & 1) ? +CUBE_HX : -CUBE_HX,
        (i & 2) ? +CUBE_HY : -CUBE_HY,
        (i & 4) ? +CUBE_HZ : -CUBE_HZ
    };
}

// Vertex indices for each face (CCW when viewed from outside)
static const int FACES[6][4] = {
    {0,1,3,2},   // bottom  Y-
    {4,6,7,5},   // top     Y+
    {0,4,5,1},   // front   Z-
    {2,3,7,6},   // back    Z+
    {0,2,6,4},   // left    X-
    {1,5,7,3},   // right   X+
};

// Face base colours (BGR)
static const cv::Scalar FACE_COLORS[6] = {
    {  50, 180,  50},   // bottom — green
    { 200,  50,  50},   // top    — blue
    {  50,  50, 200},   // front  — red
    { 180, 180,  50},   // back   — cyan-ish
    { 180,  50, 180},   // left   — magenta
    {  50, 180, 180},   // right  — yellow
};

static const int EDGES[12][2] = {
    {0,1},{2,3},{4,5},{6,7},
    {0,2},{1,3},{4,6},{5,7},
    {0,4},{1,5},{2,6},{3,7}
};

/* ------------------------------------------------------------------ */
/*  Draw the 3-D box                                                    */
/* ------------------------------------------------------------------ */
static void drawBox(cv::Mat &canvas, float roll, float pitch, float yaw) {

    // Apply camera elevation so the box isn't viewed perfectly edge-on
    auto applyPose = [&](Vec3 v) -> Vec3 {
        v = rotX(v, pitch);
        v = rotZ(v, roll);
        v = rotY(v, yaw);
        v = rotX(v, CAM_ELEV);   // fixed camera tilt
        return v;
    };

    // Project all 8 vertices
    cv::Point pts[8];
    Vec3      world[8];
    for (int i = 0; i < 8; i++) {
        world[i] = applyPose(makeVertex(i));
        pts[i]   = project(world[i]);
    }

    /* --- draw filled faces (back-to-front, painter's algorithm) --- */
    // Sort faces by average Z (depth)
    float faceZ[6];
    for (int f = 0; f < 6; f++) {
        faceZ[f] = 0;
        for (int k = 0; k < 4; k++)
            faceZ[f] += world[FACES[f][k]].z;
        faceZ[f] /= 4.0f;
    }
    int order[6] = {0,1,2,3,4,5};
    // simple insertion sort (only 6 elements)
    for (int i = 1; i < 6; i++) {
        for (int j = i; j > 0 && faceZ[order[j]] < faceZ[order[j-1]]; j--)
            std::swap(order[j], order[j-1]);
    }

    for (int fi = 0; fi < 6; fi++) {
        int f = order[fi];
        cv::Point poly[4] = {
            pts[FACES[f][0]], pts[FACES[f][1]],
            pts[FACES[f][2]], pts[FACES[f][3]]
        };
        // Simple lighting: dot of face normal with (0,1,0.5) light direction
        // Just darken back faces based on depth rank
        float brightness = 0.45f + 0.55f * ((float)fi / 5.0f);
        cv::Scalar col = FACE_COLORS[f] * brightness;
        cv::fillConvexPoly(canvas, poly, 4, col);
        cv::polylines(canvas, std::vector<cv::Point>(poly, poly+4),
                      true, cv::Scalar(220,220,220), 1, cv::LINE_AA);
    }

    /* --- draw edges on top for crisp wireframe look --- */
    for (auto &e : EDGES) {
        cv::line(canvas, pts[e[0]], pts[e[1]],
                 cv::Scalar(255,255,255), 1, cv::LINE_AA);
    }
}

/* ------------------------------------------------------------------ */
/*  Draw world-frame XYZ axes (small, bottom-left of 3D view)          */
/* ------------------------------------------------------------------ */
static void drawAxes(cv::Mat &canvas, float roll, float pitch, float yaw) {
    const int   ax = 80, ay = CY + 100;   // anchor pixel
    const float L  = 60.0f;               // axis length (pixels)

    auto axisEnd = [&](Vec3 dir) -> cv::Point {
        dir = rotX(dir, pitch);
        dir = rotZ(dir, roll);
        dir = rotY(dir, yaw);
        dir = rotX(dir, CAM_ELEV);
        // orthographic for small inset
        return { (int)(ax + L * dir.x), (int)(ay - L * dir.y) };
    };

    cv::Point o(ax, ay);
    cv::arrowedLine(canvas, o, axisEnd({1,0,0}), {  50, 50, 220}, 2, cv::LINE_AA, 0, 0.2);
    cv::arrowedLine(canvas, o, axisEnd({0,1,0}), {  50, 200, 50}, 2, cv::LINE_AA, 0, 0.2);
    cv::arrowedLine(canvas, o, axisEnd({0,0,1}), { 200,  50, 50}, 2, cv::LINE_AA, 0, 0.2);

    cv::putText(canvas, "X", axisEnd({1.3f,0,0}),
                cv::FONT_HERSHEY_SIMPLEX, 0.45, { 80, 80, 255}, 1, cv::LINE_AA);
    cv::putText(canvas, "Y", axisEnd({0,1.3f,0}),
                cv::FONT_HERSHEY_SIMPLEX, 0.45, { 50, 220, 50}, 1, cv::LINE_AA);
    cv::putText(canvas, "Z", axisEnd({0,0,1.3f}),
                cv::FONT_HERSHEY_SIMPLEX, 0.45, {220,  50, 50}, 1, cv::LINE_AA);
}

/* ------------------------------------------------------------------ */
/*  Horizontal angle bar  (±180°)                                       */
/* ------------------------------------------------------------------ */
static void drawAngleBar(cv::Mat &canvas, float angleDeg,
                         const char *label, int yPos,
                         cv::Scalar colour)
{
    const int BX = 50, BW = WIN_W - 100, BH = 18;
    // background track
    cv::rectangle(canvas, {BX, yPos}, {BX+BW, yPos+BH},
                  cv::Scalar(60,60,60), cv::FILLED);
    // filled portion (clamp to ±180°)
    float clamped = std::max(-180.0f, std::min(180.0f, angleDeg));
    int   fill    = (int)((clamped + 180.0f) / 360.0f * BW);
    cv::rectangle(canvas, {BX, yPos}, {BX+fill, yPos+BH}, colour, cv::FILLED);
    // centre line
    cv::line(canvas, {BX+BW/2, yPos}, {BX+BW/2, yPos+BH},
             cv::Scalar(200,200,200), 1);
    // text
    char buf[64];
    snprintf(buf, sizeof(buf), "%s: %+.1f°", label, angleDeg);
    cv::putText(canvas, buf, {BX+4, yPos+BH-4},
                cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255,255,255), 1, cv::LINE_AA);
}

/* ------------------------------------------------------------------ */
/*  Sensor data panel (top-right)                                       */
/* ------------------------------------------------------------------ */
static void drawDataPanel(cv::Mat &canvas, const mpu6500_t &imu,
                          float rollDeg, float pitchDeg, float fps)
{
    const int PX = WIN_W - 280, PY = 20, LH = 24;
    int y = PY;

    auto row = [&](const char *fmt, ...) {
        char buf[128];
        va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
        cv::putText(canvas, buf, {PX, y},
                    cv::FONT_HERSHEY_SIMPLEX, 0.48,
                    cv::Scalar(200,230,255), 1, cv::LINE_AA);
        y += LH;
    };

    // semi-transparent background panel
    cv::rectangle(canvas,
                  {PX - 10, PY - 18},
                  {WIN_W - 10, PY + LH * 11},
                  cv::Scalar(20,20,20), cv::FILLED);
    cv::rectangle(canvas,
                  {PX - 10, PY - 18},
                  {WIN_W - 10, PY + LH * 11},
                  cv::Scalar(80,80,80), 1);

    row("=== MPU-6500 ===");
    row("Accel X: %+6.3f g", imu.mpuData.ax);
    row("Accel Y: %+6.3f g", imu.mpuData.ay);
    row("Accel Z: %+6.3f g", imu.mpuData.az);
    row("---");
    row("Gyro  X: %+7.2f dps", imu.mpuData.gx);
    row("Gyro  Y: %+7.2f dps", imu.mpuData.gy);
    row("Gyro  Z: %+7.2f dps", imu.mpuData.gz);
    row("---");
    row("Temp:    %+6.2f C",   imu.mpuData.temp);
    row("Roll:    %+6.1f deg", rollDeg);
    row("Pitch:   %+6.1f deg", pitchDeg);

    // FPS counter (top-left)
    char fpsBuf[32];
    snprintf(fpsBuf, sizeof(fpsBuf), "FPS: %.1f", fps);
    cv::putText(canvas, fpsBuf, {10, 20},
                cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(150,255,150), 1, cv::LINE_AA);
}

/* ================================================================== */
/*  main                                                               */
/* ================================================================== */
int main()
{
    /* --- SPI + IMU init --- */
    spi_t     mySpi;
    mpu6500_t myImu;

    if (spi_open(&mySpi, "/dev/spidev0.0", SPI_MODE_0, 1000000, 8) != SPI_OK) {
        fprintf(stderr, "Failed to open SPI\n");
        return -1;
    }
    if (!mpu6500Init(&myImu, &mySpi, GYRO_FS_2000DPS, ACCEL_FS_8G, 0)) {
        spi_close(&mySpi);
        return -1;
    }

    /* --- OpenCV window --- */
    const char *WIN = "MPU-6500  3D Visualizer  [q / ESC = quit]";
    cv::namedWindow(WIN, cv::WINDOW_AUTOSIZE);

    cv::Mat canvas(WIN_H, WIN_W, CV_8UC3);

    /* --- Complementary filter state --- */
    float roll  = 0.0f;   // radians
    float pitch = 0.0f;
    float yaw   = 0.0f;   // yaw from gyro only (drifts — no magnetometer)

    /* --- Timing --- */
    auto  prevTime = std::chrono::steady_clock::now();
    float fps      = 0.0f;

    /* ---------------------------------------------------------------- */
    /*  Main loop                                                        */
    /* ---------------------------------------------------------------- */
    while (true) {
        /* --- timing / dt --- */
        auto  now = std::chrono::steady_clock::now();
        float dt  = std::chrono::duration<float>(now - prevTime).count();
        prevTime  = now;
        if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;   // guard against stalls
        fps = 0.9f * fps + 0.1f * (1.0f / dt);

        /* --- read sensor --- */
        mpu6500Update(&myImu);

        float ax = myImu.mpuData.ax;
        float ay = myImu.mpuData.ay;
        float az = myImu.mpuData.az;
        float gx = myImu.mpuData.gx * (float)(M_PI / 180.0);  // dps → rad/s
        float gy = myImu.mpuData.gy * (float)(M_PI / 180.0);
        float gz = myImu.mpuData.gz * (float)(M_PI / 180.0);

        /* --- accelerometer angles (radians) --- */
        float accelRoll  =  atan2f(ay, az);
        float accelPitch = -atan2f(ax, sqrtf(ay*ay + az*az));

        /* --- complementary filter --- */
        roll  = CF_ALPHA * (roll  + gx * dt) + (1.0f - CF_ALPHA) * accelRoll;
        pitch = CF_ALPHA * (pitch + gy * dt) + (1.0f - CF_ALPHA) * accelPitch;
        yaw  += gz * dt;   // pure gyro integration — resets slowly

        /* --- draw background gradient --- */
        for (int row_i = 0; row_i < WIN_H; row_i++) {
            float t = (float)row_i / WIN_H;
            uint8_t b = (uint8_t)(30  + t * 10);
            uint8_t g_ch = (uint8_t)(30  + t * 10);
            uint8_t r_ch = (uint8_t)(40  + t * 20);
            canvas.row(row_i).setTo(cv::Scalar(b, g_ch, r_ch));
        }

        /* --- title --- */
        cv::putText(canvas, "MPU-6500  Real-Time 3D Orientation",
                    {WIN_W/2 - 210, WIN_H - 12},
                    cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(180,180,180), 1, cv::LINE_AA);

        /* --- 3D box --- */
        drawBox(canvas, roll, pitch, yaw);

        /* --- world axes inset --- */
        drawAxes(canvas, roll, pitch, yaw);

        /* --- angle bars at bottom --- */
        drawAngleBar(canvas, roll  * 180.0f / M_PI, "Roll ",
                     WIN_H - 80, cv::Scalar(60, 120, 220));
        drawAngleBar(canvas, pitch * 180.0f / M_PI, "Pitch",
                     WIN_H - 55, cv::Scalar(60, 200, 100));

        /* --- data panel --- */
        drawDataPanel(canvas, myImu,
                      roll  * 180.0f / M_PI,
                      pitch * 180.0f / M_PI, fps);

        /* --- show --- */
        cv::imshow(WIN, canvas);

        int key = cv::waitKey(1);
        if (key == 'q' || key == 27)   // q or ESC
            break;
    }

    cv::destroyAllWindows();
    spi_close(&mySpi);
    return 0;
}
