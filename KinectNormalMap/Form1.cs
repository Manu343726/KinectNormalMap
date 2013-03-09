using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Drawing.Imaging;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using Microsoft.Kinect;

namespace KinectNormalMap
{
    public struct SmoothSettings
    {
        public bool DepthMap;
        public bool PointCloud;
        public bool NormalMap;
        public int DepthMapCicles;
        public int PointCloudCicles;
        public int NormalMapCicles;
    }

    public enum ViewMode
    {
        NORMALMAP,
        SHADEDIMAGE
    }

    public struct ShaderSettings
    {
        public const float LIGHTOFFSET_X = -0.5f;
        public const float LIGHTOFFSET_Y = -0.5f;

        public float AmbientConstant;
        public float DiffuseConstant;
        public float SpecularConstant;
        public int SpecularPower;
        public bool UseRealColor;
        public Color BaseColor;

        public SkeletonPoint Light;

        public ShaderSettings(SkeletonPoint Light, Color BaseColor, bool UseRealColor = false, float AmbientConstant = 0, float DiffuseConstant = 1, float SpecularConstant = 1, int SpecularPower = 500)
        {
            this.AmbientConstant = AmbientConstant;
            this.DiffuseConstant = DiffuseConstant;
            this.SpecularConstant = SpecularConstant;
            this.SpecularPower = SpecularPower;
            this.UseRealColor = UseRealColor;
            this.BaseColor = BaseColor;

            this.Light = new SkeletonPoint();

            this.Light.X = Light.X + LIGHTOFFSET_X;
            this.Light.Y = Light.Y + LIGHTOFFSET_Y;
        }
    }

    public partial class Form1 : Form
    {
        private const int DEPTHIMAGE_WIDTH = 640;
        private const int DEPTHIMAGE_HEIGHT = 480;
        private const DepthImageFormat DEPTHIMAGE_FORMAT = DepthImageFormat.Resolution640x480Fps30;

        private const int COLORIMAGE_WIDTH = 640;
        private const int COLORIMAGE_HEIGHT = 480;
        private const ColorImageFormat COLORIMAGE_FORMAT = ColorImageFormat.RgbResolution640x480Fps30;

        private const float LIGHTTRASLATION_FACTOR = 0.1f;

        private KinectSensor kinect;

        private CoordinateMapper mapper;

        private Bitmap normalMap;
        private Bitmap shadedImage;

        private byte[] normalMapBytes;
        private byte[] shadedImageBytes;

        private ColorImagePoint[] colorPixels;
        private DepthImagePixel[] depthPixels;
        private SkeletonPoint[] pointCloud;

        private ShaderSettings shaderSettings;

        private SmoothSettings smoothConfig;
        private ViewMode viewMode;

        private bool processing;

        private delegate void invokeDelegate();

        public Form1()
        {
            InitializeComponent();
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            if (KinectStartup())
            {
                this.kinect.DepthStream.Enable(DEPTHIMAGE_FORMAT);
                //this.kinect.SkeletonStream.Enable();

                this.depthPixels = new DepthImagePixel[this.kinect.DepthStream.FramePixelDataLength];
                this.pointCloud = new SkeletonPoint[this.kinect.DepthStream.FramePixelDataLength];
                this.shaderSettings = new ShaderSettings(new SkeletonPoint(), Color.White,false,0,1,0,1);

                this.mapper = new CoordinateMapper(this.kinect);

                this.kinect.AllFramesReady += AllFramesReady;
                //this.kinect.SkeletonFrameReady += skeletonFrameReady;
                //this.kinect.DepthFrameReady += kinectDepthFrameReady;
                this.processing = false;

                this.normalMap = new Bitmap(DEPTHIMAGE_WIDTH, DEPTHIMAGE_HEIGHT, PixelFormat.Format24bppRgb);
                this.shadedImage = new Bitmap(COLORIMAGE_WIDTH, COLORIMAGE_HEIGHT, PixelFormat.Format24bppRgb);
                this.Size = new Size(DEPTHIMAGE_WIDTH, DEPTHIMAGE_HEIGHT);

                this.smoothConfig = new SmoothSettings();
                this.smoothConfig.DepthMapCicles = this.smoothConfig.PointCloudCicles = this.smoothConfig.NormalMapCicles = 1;
                this.viewMode = ViewMode.SHADEDIMAGE;

                this.BackgroundImage = this.normalMap;

                try { this.kinect.Start(); }
                catch (System.IO.IOException)
                {
                    MessageBox.Show("Kinect device is already in use!", "KinectNormalMap - ERROR", MessageBoxButtons.OK, MessageBoxIcon.Error);
                    this.Close();
                    Application.Exit();
                }


            }
            else
            {
                MessageBox.Show("Kinect device is not plugged!", "KinectNormalMap - ERROR", MessageBoxButtons.OK, MessageBoxIcon.Error);
                this.Close();
                Application.Exit();
            }
        }

        private void Form1_FormClosed(object sender, FormClosedEventArgs e)
        {
            base.Dispose();

            if (this.kinect != null)
            {
                if (this.kinect.IsRunning)
                    this.kinect.Stop();

                this.kinect = null;
            }
        }

        private bool KinectStartup()
        {
            this.kinect = null;

            foreach (KinectSensor potentialSensor in KinectSensor.KinectSensors)
            {
                if (potentialSensor.Status == KinectStatus.Connected)
                {
                    this.kinect = potentialSensor;
                    break;
                }
            }

            return (this.kinect != null);
        }

        private void skeletonFrameReady(Object sender, SkeletonFrameReadyEventArgs args)
        {
            SkeletonFrame frame = args.OpenSkeletonFrame();

            if (frame != null)
            {
                Skeleton[] skeletons = new Skeleton[frame.SkeletonArrayLength];
                Skeleton skeleton = null;

                frame.CopySkeletonDataTo(skeletons);

                foreach (Skeleton potentialSkeleton in skeletons)
                {
                    if (potentialSkeleton.TrackingState == SkeletonTrackingState.Tracked)
                    {
                        skeleton = potentialSkeleton;
                        break;
                    }
                }

                if (skeleton != null)
                {
                    this.shaderSettings.Light = skeleton.Joints[JointType.HandLeft].Position;
                }
            }
        }

        private void kinectDepthFrameReady(object sender, DepthImageFrameReadyEventArgs args)
        {
            if (!this.processing && DepthToPointCloud(args.OpenDepthImageFrame()))
            {
                Thread thread;
                this.processing = true;

                thread = new Thread(new ThreadStart(() =>
                    {
                        ProcessNormals();

                        this.processing = false;
                    }));

                thread.Start();
            }
        }

        private void AllFramesReady(object sender, AllFramesReadyEventArgs args)
        {
            if (!this.processing && DepthToPointCloud(args.OpenDepthImageFrame()))
            {
                Thread thread;
                this.processing = true;

                thread = new Thread(new ThreadStart(() =>
                {
                    ProcessNormals();

                    DrawVector(this.kinect.AccelerometerGetCurrentReading());

                    try
                    {
                        this.Invoke(new invokeDelegate(() =>
                        {
                            if (this.viewMode == ViewMode.SHADEDIMAGE)
                                this.BackgroundImage = this.shadedImage;
                            else
                                this.BackgroundImage = this.normalMap;

                            this.Refresh();
                        }));
                    }
                    catch (InvalidOperationException) { };

                    this.processing = false;
                }));

                thread.Start();
            }
        }

        private Vector4 GetFloorNormal(Tuple<float, float, float, float> FloorClipPlane)
        {
            Vector4 floorNormalVector = new Vector4();

            floorNormalVector.X = FloorClipPlane.Item1;
            floorNormalVector.Y = FloorClipPlane.Item2;
            floorNormalVector.Z = FloorClipPlane.Item3;
            floorNormalVector.W = FloorClipPlane.Item4;

            return floorNormalVector;
        }

        private bool DepthToPointCloud(DepthImageFrame frame)
        {
            if (frame != null)
            {
                this.depthPixels = new DepthImagePixel[frame.PixelDataLength];
                frame.CopyDepthImagePixelDataTo(this.depthPixels);

                if (this.smoothConfig.DepthMap)
                {
                    DepthImagePixel[] tempDepthImage = new DepthImagePixel[frame.PixelDataLength];

                    for (int i = 0; i < this.smoothConfig.DepthMapCicles; ++i)
                    {
                        Parallel.For(0, DEPTHIMAGE_WIDTH, (int x) =>
                        {
                            Parallel.For(0, DEPTHIMAGE_HEIGHT, (int y) =>
                            {
                                int index = x + (DEPTHIMAGE_WIDTH * y);

                                if (x > 0 && y > 0 && x < DEPTHIMAGE_WIDTH - 1 && y < DEPTHIMAGE_HEIGHT - 1)//Caso común: Dentro de la imagen
                                {
                                    tempDepthImage[index].Depth = (short)((this.depthPixels[index].Depth //Él
                                                             + this.depthPixels[index - DEPTHIMAGE_WIDTH - 1].Depth //Arriba iDepthquierda
                                                             + this.depthPixels[index - DEPTHIMAGE_WIDTH].Depth //Arriba
                                                             + this.depthPixels[index - DEPTHIMAGE_WIDTH + 1].Depth //Arriba derecha
                                                             + this.depthPixels[index + 1].Depth //Derecha
                                                             + this.depthPixels[index + DEPTHIMAGE_WIDTH + 1].Depth //Abajo derecha
                                                             + this.depthPixels[index + DEPTHIMAGE_WIDTH].Depth //Abajo
                                                             + this.depthPixels[index + DEPTHIMAGE_WIDTH - 1].Depth //Abajo iDepthquierda
                                                             + this.depthPixels[index - 1].Depth //Izquierda
                                                             ) / 9);
                                }
                                else
                                    tempDepthImage[index] = this.depthPixels[index];
                            });
                        });


                    }

                    mapper.MapDepthFrameToSkeletonFrame(DEPTHIMAGE_FORMAT, tempDepthImage, this.pointCloud);
                }
                else
                    mapper.MapDepthFrameToSkeletonFrame(DEPTHIMAGE_FORMAT, this.depthPixels, this.pointCloud);

                return true;
            }
            else
                return false;
        }

        private void SmoothPointCloud()
        {
            for (int i = 0; i < this.smoothConfig.PointCloudCicles; ++i)
            {
                SkeletonPoint[] tempPoints = new SkeletonPoint[DEPTHIMAGE_WIDTH * DEPTHIMAGE_HEIGHT];

                Parallel.For(0, DEPTHIMAGE_WIDTH, (int x) =>
                {
                    Parallel.For(0, DEPTHIMAGE_HEIGHT, (int y) =>
                    {
                        int index = x + (DEPTHIMAGE_WIDTH * y);

                        if (x > 0 && y > 0 && x < DEPTHIMAGE_WIDTH - 1 && y < DEPTHIMAGE_HEIGHT - 1)//Caso común: Dentro de la imagen
                        {
                            tempPoints[index].X = this.pointCloud[index].X;
                            tempPoints[index].Y = this.pointCloud[index].Y;

                            tempPoints[index].Z = (float)((this.pointCloud[index].Z //Él
                                                     + this.pointCloud[index - DEPTHIMAGE_WIDTH - 1].Z //Arriba izquierda
                                                     + this.pointCloud[index - DEPTHIMAGE_WIDTH].Z //Arriba
                                                     + this.pointCloud[index - DEPTHIMAGE_WIDTH + 1].Z //Arriba derecha
                                                     + this.pointCloud[index + 1].Z //Derecha
                                                     + this.pointCloud[index + DEPTHIMAGE_WIDTH + 1].Z //Abajo derecha
                                                     + this.pointCloud[index + DEPTHIMAGE_WIDTH].Z //Abajo
                                                     + this.pointCloud[index + DEPTHIMAGE_WIDTH - 1].Z //Abajo izquierda
                                                     + this.pointCloud[index - 1].Z //izquierda
                                                     ) / 9);
                        }
                        else
                            tempPoints[index] = this.pointCloud[index];
                    });
                });

                this.pointCloud = tempPoints;
            }
        }

        private void SmoothNormalMap()
        {
            byte[] tempBytes = new byte[this.normalMapBytes.Length];

            for (int i = 0; i < this.smoothConfig.NormalMapCicles; ++i)
            {
                Parallel.For(0, DEPTHIMAGE_WIDTH, (int x) =>
                {
                    Parallel.For(0, DEPTHIMAGE_HEIGHT, (int y) =>
                    {
                        int index = x + (y * DEPTHIMAGE_WIDTH);

                        if (x > 0 && y > 0 && x < DEPTHIMAGE_WIDTH - 1 && y < DEPTHIMAGE_HEIGHT - 1)//Caso común: Dentro de la imagen
                        {
                            //Rojo:
                            tempBytes[(index * 3)] = (byte)((this.normalMapBytes[(index * 3)] //Él
                                                     + this.normalMapBytes[(index * 3) - (DEPTHIMAGE_WIDTH * 3) - 3] //Arriba izquierda
                                                     + this.normalMapBytes[(index * 3) - (DEPTHIMAGE_WIDTH * 3)] //Arriba
                                                     + this.normalMapBytes[(index * 3) - (DEPTHIMAGE_WIDTH * 3) + 3] //Arriba derecha
                                                     + this.normalMapBytes[(index * 3) + 3] //Derecha
                                                     + this.normalMapBytes[(index * 3) + (DEPTHIMAGE_WIDTH * 3) + 3] //Abajo derecha
                                                     + this.normalMapBytes[(index * 3) + (DEPTHIMAGE_WIDTH * 3)] //Abajo
                                                     + this.normalMapBytes[(index * 3) + (DEPTHIMAGE_WIDTH * 3) - 3] //Abajo izquierda
                                                     + this.normalMapBytes[(index * 3) - 3] //izquierda
                                                     ) / 9);

                            //Verde:
                            tempBytes[(index * 3) + 1] = (byte)((this.normalMapBytes[(index * 3) + 1] //Él
                                                     + this.normalMapBytes[(index * 3) - (DEPTHIMAGE_WIDTH * 3) - 2] //Arriba izquierda
                                                     + this.normalMapBytes[(index * 3) - (DEPTHIMAGE_WIDTH * 3) + 1] //Arriba
                                                     + this.normalMapBytes[(index * 3) - (DEPTHIMAGE_WIDTH * 3) + 4] //Arriba derecha
                                                     + this.normalMapBytes[(index * 3) + 4] //Derecha
                                                     + this.normalMapBytes[(index * 3) + (DEPTHIMAGE_WIDTH * 3) + 4] //Abajo derecha
                                                     + this.normalMapBytes[(index * 3) + (DEPTHIMAGE_WIDTH * 3) + 1] //Abajo
                                                     + this.normalMapBytes[(index * 3) + (DEPTHIMAGE_WIDTH * 3) - 2] //Abajo izquierda
                                                     + this.normalMapBytes[(index * 3) - 2] //izquierda
                                                     ) / 9);

                            //Azul:
                            tempBytes[(index * 3) + 2] = (byte)((this.normalMapBytes[(index * 3) + 2] //Él
                                                     + this.normalMapBytes[(index * 3) - (DEPTHIMAGE_WIDTH * 3) - 1] //Arriba izquierda
                                                     + this.normalMapBytes[(index * 3) - (DEPTHIMAGE_WIDTH * 3) + 2] //Arriba
                                                     + this.normalMapBytes[(index * 3) - (DEPTHIMAGE_WIDTH * 3) + 5] //Arriba derecha
                                                     + this.normalMapBytes[(index * 3) + 5] //Derecha
                                                     + this.normalMapBytes[(index * 3) + (DEPTHIMAGE_WIDTH * 3) + 5] //Abajo derecha
                                                     + this.normalMapBytes[(index * 3) + (DEPTHIMAGE_WIDTH * 3) + 2] //Abajo
                                                     + this.normalMapBytes[(index * 3) + (DEPTHIMAGE_WIDTH * 3) - 1] //Abajo izquierda
                                                     + this.normalMapBytes[(index * 3) - 1] //izquierda
                                                     ) / 9);
                        }
                        else
                        {
                            tempBytes[index * 3] = this.normalMapBytes[index * 3];
                            tempBytes[index * 3 + 1] = this.normalMapBytes[index * 3 + 1];
                            tempBytes[index * 3 + 2] = this.normalMapBytes[index * 3 + 2];
                        }
                    });
                });

                this.normalMapBytes = tempBytes;
            }
        }

        private void ProcessNormals()
        {
            int byteArrayLenght;

            BitmapData data =
            this.normalMap.LockBits(new Rectangle(0, 0, DEPTHIMAGE_WIDTH, DEPTHIMAGE_HEIGHT), System.Drawing.Imaging.ImageLockMode.ReadWrite, PixelFormat.Format24bppRgb);

            byteArrayLenght = Math.Abs(data.Stride) * DEPTHIMAGE_HEIGHT;
            this.normalMapBytes = new byte[byteArrayLenght];

            if (this.smoothConfig.PointCloud) SmoothPointCloud();

            Parallel.For(0, DEPTHIMAGE_WIDTH, (int x) =>
            {
                Parallel.For(0, DEPTHIMAGE_HEIGHT, (int y) =>
                {
                    int index = x + (DEPTHIMAGE_WIDTH * y);
                    SkeletonPoint point = this.pointCloud[index];
                    Vector4 normal = new Vector4();

                    if (KinectSensor.IsKnownPoint(point))
                        normal = CalculateNormal(x, y, index);

                    normal = Normalize(normal);

                    this.normalMapBytes[(index * 3)] = (byte)(128 + (normal.X * 128));//Rojo
                    this.normalMapBytes[(index * 3) + 1] = (byte)(128 + (normal.Y * 128));//Verde
                    this.normalMapBytes[(index * 3) + 2] = (byte)(128 + (normal.Z * 128));//Azul
                });
            });

            if (this.smoothConfig.NormalMap) SmoothNormalMap();

            if (this.viewMode == ViewMode.SHADEDIMAGE) Shader();

            System.Runtime.InteropServices.Marshal.Copy(this.normalMapBytes, 0, data.Scan0, byteArrayLenght);
            this.normalMap.UnlockBits(data);
        }

        private Color GetRealColor(int x, int y)
        {
            return Color.White;
        }

        private void Shader()
        {
            BitmapData data =
            this.shadedImage.LockBits(new Rectangle(0, 0, COLORIMAGE_WIDTH, COLORIMAGE_HEIGHT), ImageLockMode.WriteOnly, PixelFormat.Format24bppRgb);

            int byteArrayLenght = Math.Abs(data.Stride) * COLORIMAGE_HEIGHT;
            this.shadedImageBytes = new byte[byteArrayLenght];

            Parallel.For(0, DEPTHIMAGE_WIDTH, (int x) =>
            {
                Parallel.For(0, DEPTHIMAGE_HEIGHT, (int y) =>
                {
                    int index = x + (y * DEPTHIMAGE_WIDTH);

                    SkeletonPoint point = this.pointCloud[index];

                    if (KinectSensor.IsKnownPoint(point))
                    {
                        Color pointColor = this.shaderSettings.UseRealColor ? GetRealColor(x, y) : this.shaderSettings.BaseColor;
                        Vector4 normal = new Vector4();
                        Vector4 view = new Vector4();
                        Vector4 ray = new Vector4();
                        Vector4 reflection = new Vector4();
                        float ambient, difusse, specular;

                        normal.X = ((float)this.normalMapBytes[index * 3] - 128f) / 128f;
                        normal.Y = ((float)this.normalMapBytes[index * 3 + 1] - 128f) / 128f;
                        normal.Z = ((float)this.normalMapBytes[index * 3 + 2] - 128f) / 128f;

                        view.X = point.X;
                        view.Y = point.Y;
                        view.Z = point.Z;
                        view = Normalize(view);

                        ray.X = point.X - this.shaderSettings.Light.X;
                        ray.Y = point.Y - this.shaderSettings.Light.Y;
                        ray.Z = point.Z - this.shaderSettings.Light.Z;

                        ray = Normalize(ray);

                        float NL = ((normal.X * ray.X) + (normal.Y * ray.Y) + (normal.Z * ray.Z));
                        reflection.X = 2 * normal.X * NL - ray.X;
                        reflection.Y = 2 * normal.Y * NL - ray.Y;
                        reflection.Z = 2 * normal.Z * NL - ray.Z;
                        reflection = Normalize(reflection);

                        ambient = this.shaderSettings.AmbientConstant;
                        difusse = this.shaderSettings.DiffuseConstant * NL;

                        float RV = ((reflection.X * view.X) + (reflection.Y * view.Y) + (reflection.Z * view.Z));

                        if (RV < 0)
                            specular = (float)Math.Abs(this.shaderSettings.SpecularConstant * Math.Pow(RV, this.shaderSettings.SpecularPower));
                        else
                            specular = 0;

                        this.shadedImageBytes[index * 3] = (byte)Math.Max(Math.Min((pointColor.B * (ambient + difusse + specular)), 255), 0);//rojo
                        this.shadedImageBytes[index * 3 + 1] = (byte)Math.Max(Math.Min((pointColor.G * (ambient + difusse + specular)), 255), 0);//verde
                        this.shadedImageBytes[index * 3 + 2] = (byte)Math.Max(Math.Min((pointColor.R * (ambient + difusse + specular)), 255), 0);//azul
                    }
                    else
                    {
                        this.shadedImageBytes[index * 3] = 0;//rojo
                        this.shadedImageBytes[index * 3 + 1] = 0;//verde
                        this.shadedImageBytes[index * 3 + 2] = 0;//azul
                    }
                });
            });

            System.Runtime.InteropServices.Marshal.Copy(this.shadedImageBytes, 0, data.Scan0, byteArrayLenght);
            this.shadedImage.UnlockBits(data);
        }

        private Vector4 CalculateNormal(int x, int y, int index)
        {
            Vector4 normal = new Vector4();
            SkeletonPoint[] neighbours = new SkeletonPoint[3];
            bool[] validNeighbours = new bool[3];

            if (x > 0 && y > 0 && x < DEPTHIMAGE_WIDTH - 1 && y < DEPTHIMAGE_HEIGHT - 1)//Caso común: Dentro de la imagen
            {
                //Én triángulo:

                //     #
                //     O
                //   #   #

                neighbours[0] = this.pointCloud[index - DEPTHIMAGE_WIDTH];     //Superior
                neighbours[1] = this.pointCloud[index + DEPTHIMAGE_WIDTH - 1]; //Inferior izquierdo
                neighbours[2] = this.pointCloud[index + DEPTHIMAGE_WIDTH + 1]; //Inferior derecho

                validNeighbours[0] = KinectSensor.IsKnownPoint(neighbours[0]);
                validNeighbours[1] = KinectSensor.IsKnownPoint(neighbours[1]);
                validNeighbours[2] = KinectSensor.IsKnownPoint(neighbours[2]);

                if (validNeighbours[0] && validNeighbours[1] && validNeighbours[2])
                {
                    //normal = DotProduct(Vector(neighbours[0], neighbours[1]), Vector(neighbours[0], neighbours[2]));
                    normal = DotProduct(neighbours);
                }
                else
                {
                    normal.X = this.pointCloud[index].X;
                    normal.Y = this.pointCloud[index].Y;
                    normal.Z = this.pointCloud[index].Z;
                }
            }
            else
            {
                normal.X = 0;
                normal.Y = 0;
                normal.Z = 0;
            }

            return normal;
        }

        private Vector4 Normalize(Vector4 normal)
        {
            if (normal.X != 0 || normal.Y != 0 || normal.Z != 0)
            {
                float lenght = (float)Math.Sqrt((normal.X * normal.X) + (normal.Y * normal.Y) + (normal.Z * normal.Z));

                normal.X /= lenght;
                normal.Y /= lenght;
                normal.Z /= lenght;
            }

            return normal;
        }

        private Vector4 DotProduct(SkeletonPoint[] points)
        {
            Vector4 vector = new Vector4();

            vector.X = ((points[1].Y - points[0].Y) * (points[2].Z - points[0].Z)) - ((points[2].Y - points[0].Y) - (points[1].Z - points[0].Z));
            vector.Y = ((points[1].Z - points[0].Z) * (points[2].X - points[0].X)) - ((points[2].Z - points[0].Z) - (points[1].X - points[0].X));
            vector.Z = ((points[1].X - points[0].X) * (points[2].Y - points[0].Y)) - ((points[2].X - points[0].X) * (points[1].Y - points[0].Y));

            return vector;
        }

        private Vector4 DotProduct(Vector4 V1, Vector4 V2)
        {
            Vector4 vector = new Vector4();

            vector.X = (V1.Y * V2.Z) - (V2.Y - V1.Z);
            vector.Y = (V1.Z * V2.X) - (V2.Z - V1.X);
            vector.Z = (V1.X * V2.Y) - (V2.X * V1.Y);

            return vector;
        }

        private Vector4 Vector(SkeletonPoint P1, SkeletonPoint P2)
        {
            Vector4 vector = new Vector4();

            vector.X = P2.X - P1.X;
            vector.Y = P2.Y - P1.Y;
            vector.Z = P2.Z - P1.Z;

            return vector;
        }

        private void DrawBoneVector(Joint J1, Joint J2)
        {
            Vector4 vector = new Vector4();

            vector.X = J2.Position.X - J1.Position.X;
            vector.Y = J2.Position.Y - J1.Position.Y;
            vector.Z = J2.Position.Z - J1.Position.Z;

            DrawVector(vector);
        }

        private void DrawVector(Vector4 vector)
        {
            Graphics g;
            Color color = Color.FromArgb(ParseNormaltoRGB(vector));

            if(this.viewMode==ViewMode.NORMALMAP)
                g = Graphics.FromImage(this.normalMap);
            else
                g = Graphics.FromImage(this.shadedImage);

            vector = Normalize(vector);

            g.FillRectangle(new SolidBrush(color), 0, 0, (int)(DEPTHIMAGE_WIDTH * 0.25), (int)(DEPTHIMAGE_HEIGHT * 0.25));

            g.FillRectangle(Brushes.Red, 0, (int)(DEPTHIMAGE_HEIGHT * 0.25 * 0.25), (int)(color.R * DEPTHIMAGE_HEIGHT * 0.25 / 255), (int)(DEPTHIMAGE_HEIGHT * 0.25 * 0.25));
            g.FillRectangle(Brushes.Green, 0, (int)(DEPTHIMAGE_HEIGHT * 0.25 * 0.50), (int)(color.G * DEPTHIMAGE_HEIGHT * 0.25 / 255), (int)(DEPTHIMAGE_HEIGHT * 0.25 * 0.25));
            g.FillRectangle(Brushes.Blue, 0, (int)(DEPTHIMAGE_HEIGHT * 0.25 * 0.75), (int)(color.B * DEPTHIMAGE_HEIGHT * 0.25 / 255), (int)(DEPTHIMAGE_HEIGHT * 0.25 * 0.25));

            g.DrawRectangle(Pens.Black, 0, 0, (int)(DEPTHIMAGE_WIDTH * 0.25), (int)(DEPTHIMAGE_HEIGHT * 0.25));

            g.DrawString(vector.X.ToString("0.00") + " "
                       + vector.Y.ToString("0.00") + " "
                       + vector.Z.ToString("0.00") + "\n"

                       + "RGB(" + color.R.ToString() + ","
                       + color.G.ToString() + ","
                       + color.B.ToString() + ")",
                       this.Font, Brushes.White, 2, 2, StringFormat.GenericDefault);
        }

        private int ParseNormaltoRGB(Vector4 normal)
        {
            return (255 << 24) |                            //Alfa
                   ((byte)(128 + (normal.X * 128)) << 16) | //Rojo
                   ((byte)(128 + (normal.Y * 128)) << 8) |  //Verde
                   ((byte)(128 + (normal.Z * 128)) << 0);   //Azul
        }

        private void Form1_KeyDown(object sender, KeyEventArgs e)
        {
            switch (e.KeyCode)
            {
                case Keys.D:
                    this.smoothConfig.DepthMap = !this.smoothConfig.DepthMap;
                    break;
                case Keys.P:
                    this.smoothConfig.PointCloud = !this.smoothConfig.PointCloud;
                    break;
                case Keys.N:
                    this.smoothConfig.NormalMap = !this.smoothConfig.NormalMap;
                    break;
                case Keys.S:
                    if (this.viewMode == ViewMode.NORMALMAP)
                        this.viewMode = ViewMode.SHADEDIMAGE;
                    else
                        this.viewMode = ViewMode.NORMALMAP;

                    break;
                case Keys.Up:
                    this.shaderSettings.Light.Z += LIGHTTRASLATION_FACTOR;
                    break;
                case Keys.Down:
                    this.shaderSettings.Light.Z -= LIGHTTRASLATION_FACTOR;
                    break;
                case Keys.Right:
                    this.shaderSettings.Light.X -= LIGHTTRASLATION_FACTOR;
                    break;
                case Keys.Left:
                    this.shaderSettings.Light.X += LIGHTTRASLATION_FACTOR;
                    break;
                case Keys.NumPad8:
                    this.shaderSettings.Light.Y += LIGHTTRASLATION_FACTOR;
                    break;
                case Keys.NumPad2:
                    this.shaderSettings.Light.Y -= LIGHTTRASLATION_FACTOR;
                    break;
                case Keys.Escape:
                    this.Close();
                    Application.Exit();
                    break;
                case Keys.Add:
                    this.smoothConfig.DepthMapCicles++;
                    this.smoothConfig.PointCloudCicles++;
                    this.smoothConfig.NormalMapCicles++;
                    break;
                case Keys.Subtract:
                    if (this.smoothConfig.DepthMapCicles > 1)
                    {
                        this.smoothConfig.DepthMapCicles--;
                        this.smoothConfig.PointCloudCicles--;
                        this.smoothConfig.NormalMapCicles--;
                    }
                    break;
            }
        }
    }
}
