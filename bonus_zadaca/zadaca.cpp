#include <limits>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include "geometry.h"

void swap(float& a, float& b){
  float c = a;
  a = b;
  b = c;
}

// struktura
typedef std::vector<Vec3f> Image;

struct Light
{
    Vec3f position;
    float intensity;
    
    Light(const Vec3f &position, const float &intensity) : position(position), intensity(intensity) {}
};
typedef std::vector<Light> Lights;

struct Material
{
    Vec2f albedo; // difuzni i spekularni koeficijenti refleksije
    Vec3f diffuse_color;
    float specular_exponent;
    
    Material(const Vec2f &a, const Vec3f &color, const float &coef) : albedo(a), diffuse_color(color), specular_exponent(coef) {}
    Material() : albedo(Vec2f(1, 0)), diffuse_color(), specular_exponent(1.f) {}
};

struct BoundingBox
{
    Vec3f a, b;
    BoundingBox(const Vec3f a, const Vec3f b): a(a), b(b) {}
    bool hitbox(const Vec3f &p, const Vec3f &d) {
        float ta = (a.x - p.x) / d.x; 
        float tb = (b.x - p.x) / d.x; 
    
        if (ta > tb) swap(ta, tb); 
    
        float tya = (a.y - p.y) / d.y; 
        float tyb = (b.y - p.y) / d.y; 
    
        if (tya > tyb) swap(tya, tyb);
        if ((ta > tyb) || (tya > tb)) return false;
        if (tya > ta) ta = tya;
        if (tyb < tb) tb = tyb;
    
        float tza = (a.z - p.z) / d.z; 
        float tzb = (b.z - p.z) / d.z; 
    
        if (tza > tzb) swap(tza, tzb); 
        if ((ta > tzb) || (tza > tb)) return false; 

        return true; 
    }
    BoundingBox combine(const BoundingBox &n, const BoundingBox &m){
        // napravi novi volumen od ova dva
        float min_x = std::min(std::min(n.a.x, n.b.x), std::min(m.a.x, m.b.x));
        float min_y = std::min(std::min(n.a.y, n.b.y), std::min(m.a.y, m.b.y));
        float min_z = std::min(std::min(n.a.z, n.b.z), std::min(m.a.z, m.b.z));
        float max_x = std::max(std::max(n.a.x, n.b.x), std::max(m.a.x, m.b.x));
        float max_y = std::max(std::max(n.a.y, n.b.y), std::max(m.a.y, m.b.y));
        float max_z = std::max(std::max(n.a.z, n.b.z), std::max(m.a.z, m.b.z));
        return BoundingBox(Vec3f(min_x, min_y, min_z), Vec3f(max_x, max_y, max_z));
    }
};

struct Object
{
    Material material;
    virtual bool ray_intersect(const Vec3f &p, const Vec3f &d, float &t) const = 0;
    virtual Vec3f normal(const Vec3f &p) const = 0;    
};
typedef std::vector<Object*> Objects;

struct Triangle : Object
{
    Vec3f a, b, c;
    
    Triangle(const Vec3f &a, const Vec3f &b, const Vec3f &c, const Material &m) : a(a), b(b), c(c)
    {
        Object::material = m;
    }
    
    bool ray_intersect(const Vec3f &p, const Vec3f &d, float &t) const
    { //a je l
        Vec3f edge1 = b - a;
        Vec3f edge2 = c - a;
        Vec3f h = cross(d, edge2);
        float l = edge1*h;
        if (l > -0.000001 && l < 0.000001) return false;
        float f = 1.0/l;
        Vec3f s = p - a;
        float u = f * (s * h);
        if (u < 0.0 || u > 1.0) return false;
        Vec3f q = cross(s, edge1);
        float v = f * (d * q);
        if (v < 0.0 || u + v > 1.0) return false;
        t = f * (edge2*q);
        return t > 0.000001;
    }
    
    Vec3f normal(const Vec3f &p) const
    {
         return cross(a - b, c - b).normalize();
    }
};

struct Sphere : Object
{
    Vec3f c; // centar
    float r; // radius
    
    Sphere(const Vec3f &c, const float &r, const Material &m) : c(c), r(r)
    {
        Object::material = m;
    }
    
    bool ray_intersect(const Vec3f &p, const Vec3f &d, float &t) const
    {
        Vec3f v = c - p; // vektor izmedju izvora zrake i centra
        
        if (d * v < 0) // skalarni produkt
        {
            // sfera se nalazi iza zrake
            return false;
        }
        else
        {
            // izracunaj projekciju
            Vec3f pc = p + d * ((d * v)/d.norm());
            if ((c - pc)*(c - pc) > r*r)
            {
                // nema sjeciste
                return false;                
            }
            else
            {
                float dist = sqrt(r*r - (c - pc) * (c - pc));
                
                if (v*v > r*r) // izvor pogleda izvan sfere
                {
                    t = (pc - p).norm() - dist;
                }
                else // izvor pogleda unutar sfere
                {
                    t = (pc - p).norm() + dist;                    
                }
                
                return true;
            }
        }
    }
    
    Vec3f normal(const Vec3f &p) const
    {
        return (p - c).normalize();        
    }
};

// funkcija koja koristimo za crtanje scene
bool scene_intersect(const Vec3f &orig, const Vec3f &dir, const Objects &objs, Vec3f &hit, Material &material, Vec3f &N)
{
    // inicijalno, pretp. da je predmet daleko
    float dist = std::numeric_limits<float>::max();
    float obj_dist = dist;
    
    for (auto obj : objs)
    {
        if (obj->ray_intersect(orig, dir, obj_dist) &&  obj_dist < dist)
        {
            dist = obj_dist;
            hit = orig + dir * obj_dist;
			      N = obj->normal(hit);
            material = obj->material;
        }
    }
    
    // provjeri je li sjecište predaleko
    return dist < 1000;
}


// funkcija koja vraca udaljenost sjecista pravca i sfere
Vec3f cast_ray(const Vec3f &orig, const Vec3f &dir, const Objects &objs, const Lights &lights)
{
    Vec3f hit_point;
    Vec3f hit_normal; // normala na povrsinu
    Material hit_material;
    
    if (!scene_intersect(orig, dir, objs, hit_point, hit_material, hit_normal))
    {
        return Vec3f(0.7, 0.9, 0.7); // vrati boju pozadine
    }
    else
    {
        float diffuse_light_intensity = 0;
        float specular_light_intensity = 0;

        for(auto light : lights)
        {
            Vec3f light_dir = (light.position - hit_point).normalize(); // smjer svjetla
            float light_dist = (light.position - hit_point).norm(); // udaljenost do svjetla
						
            // sjene
            Vec3f shadow_orig;
            Vec3f shadow_hit_point;
            Vec3f shadow_hit_normal;
            Material shadow_material;
            
            if (light_dir * hit_normal < 0) hit_normal = -hit_normal;
      
            // epsilon pomak od tocke sjecista
            shadow_orig = hit_point + hit_normal * 0.001;
            
            if (scene_intersect(shadow_orig, light_dir, objs, shadow_hit_point, shadow_material, shadow_hit_normal) && (shadow_hit_point - shadow_orig).norm() < light_dist)
            {
                    continue;
            }

            // sjencanje
            // lambertov model
            diffuse_light_intensity += light.intensity * std::max(0.f,light_dir * hit_normal); 
            
            // blinn-phongov model
            // smjer pogleda
            Vec3f view_dir = (orig - hit_point).normalize();
            // poluvektor
            Vec3f half_vec = (view_dir+light_dir).normalize();    
            specular_light_intensity += light.intensity * powf(std::max(0.f,half_vec * hit_normal), hit_material.specular_exponent); 
        }
        return hit_material.diffuse_color * hit_material.albedo[0] * diffuse_light_intensity  // diffuse dio
               + Vec3f(1,1,1) * hit_material.albedo[1] * specular_light_intensity;            // specular dio
    }
}

// funkcija za crtanje
void render(const Objects &objects, const Lights &lights) 
{
    // velicina slike
    const int width = 1024;
    const int height = 768;
    const int fov = 3.14159265358979323846/2.0; // pi / 2
    
    // spremnik za sliku
    Image buffer(width * height);
    
    // nacrtaj u sliku
    for (size_t j = 0; j < height; ++j)
    {
        for (size_t i = 0; i < width; ++i)
        {
            // pošalji zraku u svaki piksel
            float x =  (2 * (i + 0.5)/(float)width  - 1) * tan(fov/2.) * width/(float)height;
            float y = -(2 * (j + 0.5)/(float)height - 1) * tan(fov/2.);
            
            // definiraj smjer
            Vec3f dir = Vec3f(x, y, -1).normalize();
            
            buffer[i + j*width] = cast_ray(Vec3f(0,0,0), dir, objects, lights);
        }            
    }
    
    
    // spremanje slike u vanjsku datoteku
    std::ofstream ofs;
    ofs.open("./zadaca.ppm", std::ofstream::binary);
    // oblikuj po ppm formatu
    ofs << "P6\n" << width << " " << height << "\n255\n";
    for (size_t i = 0; i < height * width; ++i)
    {
        for (size_t j = 0; j < 3; ++j)
        {
            // skaliraj na sa [0, 1] na [0, 255]
            unsigned char color = (unsigned char)(255.f * std::max(0.f, std::min(1.f, buffer[i][j])));
            ofs << color;
        }            
    }
    // zatvori datoteku
    ofs.close();
}

int main()
{
    // definiranje materijala
    Material red = Material(Vec2f(0.6,0.3), Vec3f(1, 0, 0), 60);
    Material green = Material(Vec2f(0.6,0.3), Vec3f(0, 0.5, 0), 60);
    Material blue = Material(Vec2f(0.9,0.1), Vec3f(0, 0, 1), 10);
    Material gray = Material(Vec2f(0.9,0.1), Vec3f(0.5, 0.5, 0.5), 10);
    
    // definiranje sfera
    Sphere s1(Vec3f(-3,    0,   -16), 2, red);
    Sphere s2(Vec3f(-1.0, -1.5, -12), 2, green);
    Sphere s3(Vec3f( 1.5, -0.5, -18), 3, blue);
    Sphere s4(Vec3f( 7,    5,   -18), 4, gray);

    Triangle t1(Vec3f(-3, 0, -8), Vec3f(-1.0, -1.5, -8), Vec3f(1.5, -0.5, -8), gray);
    
    
    // definiraj objekte u sceni
    Objects objs = { &s1, &s2, &s3, &s4, &t1 };

    
    // definiraj svjetla
    Light l1 = Light(Vec3f(-20, 20, 20), 1.5);
    Light l2 = Light(Vec3f(20, 30, 20), 1.8);
    Lights lights = { l1, l2 };
    
    render(objs, lights);
    
    return 0;
}
