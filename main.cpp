#include <iostream>
#include <cmath>
#include <cstdlib>
#include <string>
#include <fstream>


//3次元ベクトルクラス
//点の位置、方向、RGBを表すのに使用される
//ベクトル同士の加減算、スカラー倍が定義されている
class Vector3 {
    public:
        float x;
        float y;
        float z;

        //コンストラクタ
        Vector3() : x(0), y(0), z(0) {};
        //引数が一つの場合は同じ値で各成分を初期化する
        Vector3(float x) : x(x), y(x), z(x) {};
        Vector3(float x, float y, float z) : x(x), y(y), z(z) {};

        //-v
        Vector3 operator-() const {
            return Vector3(-x, -y, -z);
        };

        //v + v2
        Vector3 operator+(const Vector3& v) const {
            return Vector3(x + v.x, y + v.y, z + v.z);
        };
        //v - v2
        Vector3 operator-(const Vector3& v) const {
            return Vector3(x - v.x, y - v.y, z - v.z);
        };
        //v * k
        Vector3 operator*(float k) const {
            return Vector3(k * x, k * y, k * z);
        };
        //v / k
        Vector3 operator/(float k) const {
            return Vector3(x / k, y / k, z / k);
        };

        //長さを返す
        float length() const {
            return std::sqrt(x*x + y*y + z*z);
        };
        //長さを二乗したものを返す
        float length2() const {
            return x*x + y*y + z*z;
        };
};
//k * v
Vector3 operator*(float k, const Vector3& v) {
    return Vector3(k * v.x, k * v.y, k * v.z);
}


//長さを１に正規化したベクトルを返す
Vector3 normalize(const Vector3& v) {
    return Vector3(v.x, v.y, v.z)/v.length();
}
//内積を計算する
float dot(const Vector3& v1, const Vector3& v2) {
    return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}
//外積を計算する
Vector3 cross(const Vector3& v1, const Vector3& v2) {
    return Vector3(v1.y*v2.z - v1.z*v2.y, v1.z*v2.x - v1.x*v2.z, v1.x*v2.y - v1.y*v2.x);
}
std::ostream& operator<<(std::ostream& stream, const Vector3& v) {
    stream << "(" << v.x << ", " << v.y << ", " << v.z << ")";
}


//画像クラス
//width:横幅 height:縦幅 data:画像データ
//一次元配列dataにRGBが格納されている。i行j列目のRGBはdata[j + i*height]に格納されている
class Image {
    public:
        int width;
        int height;
        Vector3 *data;

        Image(int width, int height) : width(width), height(height) {
            //配列をメモリ上に用意する
            data = new Vector3[width*height];
        };
        ~Image() {
            //使い終わったメモリを開放する
            delete[] data;
        };

        //i行j列目のRGBを返す
        Vector3 get(int i, int j) const {
            if(i < 0 || i >= height || j < 0 || j >= width)
                std::exit(1);
            return data[j + i*height];
        };
        //i行j列目にRGBを格納する
        Vector3 set(int i, int j, const Vector3& col) {
            if(i < 0 || i >= height || j < 0 || j >= width)
                std::exit(1);
            data[j + i*height] = col;
        };
        
        //ファイル名を受け取り、ppm画像として出力する
        void ppm_output(std::string filename) {
            std::ofstream file(filename);
            file << "P3" << std::endl;
            file << width << " " << height << std::endl;
            file << 255 << std::endl;
            for(int i = 0; i < height; i++) {
                for(int j = 0; j < width; j++) {
                    Vector3 col = this->get(i, j);
                    file << (int)(255*col.x) << " " << (int)(255*col.y) << " " << (int)(255*col.z) << std::endl;
                }
            }
            file.close();
        };
};


//レイクラス
//origin:レイの始点 direction:レイの方向
class Ray {
    public:
        Vector3 origin;
        Vector3 direction;
        constexpr static float tmin = 0.001f;
        constexpr static float tmax = 10000.0f;

        Ray(const Vector3& origin, const Vector3& direction) : origin(origin), direction(direction) {};

        //原点から距離tにあるレイ上の点を返す
        Vector3 operator()(float t) const {
            return origin + t*direction;
        };
};


//衝突情報クラス
//t:衝突点までの距離 hitPos:衝突点 hitNormal:衝突点における法線
class Hit {
    public:
        float t;
        Vector3 hitPos;
        Vector3 hitNormal;

        Hit() {};
};


//球クラス
//center:中心 radius:半径
class Sphere {
    public:
        Vector3 center;
        float radius;

        Sphere(const Vector3& center, float radius) : center(center), radius(radius) {};

        //受け取ったレイに対して衝突計算を行う。衝突した場合は衝突情報をresに格納し、trueを返す
        bool intersect(const Ray& ray, Hit& res) const {
            //二次方程式 ax^2 + bx + c = 0
            float a = ray.direction.length2();
            float b = 2*dot(ray.direction, ray.origin - center);
            float c = (ray.origin - center).length2() - radius*radius;
            float D = b*b - 4*a*c;
            if(D < 0) return false;

            //近い方の衝突距離を返す
            float t0 = (-b - std::sqrt(D))/(2*a);
            float t1 = (-b + std::sqrt(D))/(2+a);
            if(t0 > ray.tmax || t1 < ray.tmin)
                return false;
            float t = t0;
            if(t < ray.tmin) {
                t = t1;
                if(t > ray.tmax)
                    return false;
            }

            //衝突情報を格納する
            res.t = t;
            res.hitPos = ray(t);
            res.hitNormal = normalize(res.hitPos - center);
            return true;
        };
};


//カメラクラス
//camPos:カメラの位置 camForward:カメラの前方向 camRight:カメラの横方向 camUp:カメラの上方向
class Camera {
    public:
        Vector3 camPos;
        Vector3 camForward;
        Vector3 camRight;
        Vector3 camUp;

        //コンストラクタ
        //カメラの位置と前方向を受け取って初期化する
        Camera(const Vector3& camPos, const Vector3& camForward) : camPos(camPos), camForward(camForward) {
            camRight = -cross(camForward, Vector3(0, 1, 0));
            camUp = cross(camRight, camForward);
        };

        //(u, v)で指定される画素に対応するレイを取得する
        Ray getRay(float u, float v) const {
            return Ray(camPos, normalize(camForward + u*camRight + v*camUp));
        };
};


int main() {
    //512 * 512の画像を用意
    Image img(512, 512);

    //カメラの初期化
    Camera cam(Vector3(0, 0, 0), Vector3(0, 0, 1));

    //球を用意する
    Sphere sphere(Vector3(0, 0, 3), 1.0f);

    //画素一つ一つに対して次の計算を行う
    for(int i = 0; i < img.width; i++) {
        for(int j = 0; j < img.height; j++) {
            //(u, v)はイメージセンサー上で真ん中を原点とし、(-1, -1) ~ (1, 1)の値を取る
            float u = (2.0*i - img.height)/img.height;
            float v = (2.0*j - img.width)/img.width;
            //カメラから対応する画素のレイを取得する
            Ray ray = cam.getRay(u, v);
            Hit res;
            //衝突したら白色にする。
            if(sphere.intersect(ray, res)) {
                img.set(i, j, Vector3(1));
            }
            else {
                img.set(i, j, Vector3(0));
            }
        }
    }
    //画像をppm形式で出力
    img.ppm_output("output.ppm");
}
