#include "mainwindow.h"

#include <QHBoxLayout>
#include <QFormLayout>
#include <QLabel>
#include <QSlider>
#include <QDebug>

#include <faust/gui/UI.h>

struct FaustUI : UI
{
  using REAL = float;

  FaustUI(MainWindow& win, QFormLayout& lay, std::vector<std::array<float*, 4>>& guis)
    : window{win}
    , controls{lay}
    , guis{guis}
  {

  }

  MainWindow& window;
  QFormLayout& controls;
  std::vector<std::array<float*, 4>>& guis;

  int colorIndex = 0;
  int currentIndex = 0;

  void openTabBox(const char* label) { }
  void openHorizontalBox(const char* label) { }
  void openVerticalBox(const char* label) { }
  void closeBox() { }
  void addButton(const char* label, REAL* zone) { }
  void addCheckButton(const char* label, REAL* zone) { }
  void addVerticalSlider(const char* label, REAL* zone, REAL init, REAL min, REAL max, REAL step)
  {
    addHorizontalSlider(label, zone, init, min, max, step);
  }

  void addHorizontalSlider(const char* label, REAL* zone, REAL init, REAL min, REAL max, REAL step)
  {
    constexpr float mult = 1e3;
    if(guis.size() <= currentIndex)
    {
      auto slider = new QSlider(Qt::Horizontal);

      slider->setRange(min * mult, max * mult);
      slider->setSingleStep(step);
      slider->setValue(init);
      controls.addRow(label, slider);

      slider->connect(slider, &QSlider::valueChanged,
                      &window, [=, &w=this->window, &guis=this->guis, index=currentIndex] (int value) {
        for(float* ptr : guis[index]) {
          *ptr = value / mult;
        }
        w.updateImage();
      });

      guis.push_back({zone, nullptr, nullptr});
    }
    else
    {
      guis[currentIndex][colorIndex] = zone;
    }

    currentIndex++;
  }

  void addNumEntry(const char* label, REAL* zone, REAL init, REAL min, REAL max, REAL step) {
    qDebug() <<" addNumEntry " << label; }

  // -- passive widgets

  void addHorizontalBargraph(const char* label, REAL* zone, REAL min, REAL max) { }
  void addVerticalBargraph(const char* label, REAL* zone, REAL min, REAL max) { }

  // -- soundfiles

  void addSoundfile(const char* label, const char* filename, Soundfile** sf_zone) { }

};

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow{parent}
    , m_sourceImage{"/home/jcelerier/shashimi.jpg"}
{
  auto centralWidg = new QWidget;
  setCentralWidget(centralWidg);
  auto layout = new QHBoxLayout{centralWidg};

  m_image = new QLabel{};
  layout->addWidget(m_image);

  auto columnWidg = new QWidget;
  layout->addWidget(columnWidg);
  columnWidg->setMinimumWidth(200);
  columnWidg->setMaximumWidth(200);
  m_controls = new QFormLayout{columnWidg};

  auto faustProgram = R"_(

                      import("stdfaust.lib");

                      /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                      //
                      // A complete Stereo FX chain with:
                      //		CHORUS
                      //		PHASER
                      //		DELAY
                      //		REVERB
                      //
                      // Designed to use the Analog Input for parameters controls.
                      //
                      // CONTROLES ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                      //
                      // ANALOG IN:
                      // ANALOG 0	: Chorus Depth
                      // ANALOG 1	: Chorus Delay
                      // ANALOG 2	: Phaser Dry/Wet
                      // ANALOG 3	: Phaser Frequency ratio
                      // ANALOG 4	: Delay Dry/Wet
                      // ANALOG 5	: Delay Time
                      // ANALOG 6	: Reverberation Dry/Wet
                      // ANALOG 7	: Reverberation Room size
                      //
                      // Available by OSC : (see BELA console for precise adress)
                      // Rate			: Chorus LFO modulation rate (Hz)
                      // Deviation	: Chorus delay time deviation.
                      //
                      // InvertSum	: Phaser inversion of phaser in sum. (On/Off)
                      // VibratoMode	: Phaser vibrato Mode. (On/Off)
                      // Speed		: Phaser LFO frequency
                      // NotchDepth	: Phaser LFO depth
                      // Feedback		: Phaser Feedback
                      // NotchWidth	: Phaser Notch Width
                      // MinNotch1	: Phaser Minimal frequency
                      // MaxNotch1	: Phaser Maximal Frequency
                      //
                      // Damp			: Reverberation Damp
                      // Stereo		: Reverberation Stereo Width
                      //
                      /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                      process = chorus_stereo(dmax,curdel,rate,sigma,do2,voices);

                      // CHORUS (from SAM demo lib) //////////////////////////////////////////////////////////////////////////////////////////////////////////
                      voices = 8; // MUST BE EVEN

                      pi = 4.0*atan(1.0);
                      periodic  = 1;

                      dmax = 8192;
                      curdel = dmax * vslider("Delay[BELA: ANALOG_1]", 0.5, 0, 1, 1) : si.smooth(0.999);
                      rateMax = 7.0; // Hz
                      rateMin = 0.01;
                      rateT60 = 0.15661;

                      rate = vslider("Rate", 0.5, rateMin, rateMax, 0.0001): si.smooth(ba.tau2pole(rateT60/6.91));
                      depth = vslider("Depth [BELA: ANALOG_0]", 0.5, 0, 1, 0.001) : si.smooth(ba.tau2pole(depthT60/6.91));
                      // (dept = dry/wet)

                      depthT60 = 0.15661;
                      delayPerVoice = 0.5*curdel/voices;
                      sigma = delayPerVoice * vslider("Deviation",0.5,0,1,0.001) : si.smooth(0.999);

                      do2 = depth;   // use when depth=1 means "multivibrato" effect (no original => all are modulated)

                      chorus_stereo(dmax,curdel,rate,sigma,do2,voices) =
                            _,_ <: *(1-do2),*(1-do2),(*(do2),*(do2) <: par(i,voices,voice(i)):>_,_) : ro.interleave(2,2) : +,+;
                            voice(i) = de.fdelay(dmax,min(dmax,del(i)))/(i+1)
                          with {
                             angle(i) = 2*pi*(i/2)/voices + (i%2)*pi/2;
                             voice(i) = de.fdelay(dmax,min(dmax,del(i))) * cos(angle(i));

                               del(i) = curdel*(i+1)/voices + dev(i);
                               rates(i) = rate/float(i+1);
                               dev(i) = sigma *
                                   os.oscp(rates(i),i*2*pi/voices);
                          };

                      // PHASER (from demo lib.) /////////////////////////////////////////////////////////////////////////////////////////////////////////////
                      phaserSt = _,_ <: _, _, phaser2_stereo : dry_wetST(dwPhaz)
                          with {

                              invert = checkbox("InvertSum");
                              vibr = checkbox("VibratoMode"); // In this mode you can hear any "Doppler"

                              phaser2_stereo = pf.phaser2_stereo(Notches,width,frqmin,fratio,frqmax,speed,mdepth,fb,invert);

                              Notches = 4; // Compile-time parameter: 2 is typical for analog phaser stomp-boxes

                              speed  = hslider("Speed", 0.5, 0, 10, 0.001);
                              depth  = hslider("NotchDepth", 1, 0, 1, 0.001);
                              fb     = hslider("Feedback", 0.7, -0.999, 0.999, 0.001);

                              width  = hslider("NotchWidth",1000, 10, 5000, 1);
                              frqmin = hslider("MinNotch1",100, 20, 5000, 1);
                              frqmax = hslider("MaxNotch1",800, 20, 10000, 1) : max(frqmin);
                              fratio = hslider("NotchFreqRatio[BELA: ANALOG_3]",1.5, 1.1, 4, 0.001);
                              dwPhaz = vslider("dryWetPhaser[BELA: ANALOG_2]", 0.5, 0, 1, 0.001);

                              mdepth = select2(vibr,depth,2); // Improve "ease of use"
                          };

                      // DELAY (with feedback and crossfeeback) //////////////////////////////////////////////////////////////////////////////////////////////
                      delay = ba.sec2samp(hslider("delay[BELA: ANALOG_5]", 1,0,2,0.001));
                      preDelL	= delay/2;
                      delL	= delay;
                      delR	= delay;

                      crossLF	= 1200;

                      CrossFeedb = 0.6;
                      dwDel = vslider("dryWetDelay[BELA: ANALOG_4]", 0.5, 0, 1, 0.001);

                      routeur(a,b,c,d) = ((a*CrossFeedb):fi.lowpass(2,crossLF))+c,
                                ((b*CrossFeedb):fi.lowpass(2,crossLF))+d;

                      xdelay = _,_ <: _,_,((de.sdelay(65536, 512,preDelL),_):
                          (routeur : de.sdelay(65536, 512,delL) ,de.sdelay(65536, 512,delR)) ~ (_,_)) : dry_wetST(dwDel);

                      // REVERB (from freeverb_demo) /////////////////////////////////////////////////////////////////////////////////////////////////////////
                      reverb = _,_ <: (*(g)*fixedgain, *(g)*fixedgain :
                        re.stereo_freeverb(combfeed, allpassfeed, damping, spatSpread)),
                        *(1-g), *(1-g) :> _,_
                          with {
                              scaleroom   = 0.28;
                              offsetroom  = 0.7;
                              allpassfeed = 0.5;
                              scaledamp   = 0.4;
                              fixedgain   = 0.1;
                              origSR = 44100;

                              damping = vslider("Damp",0.5, 0, 1, 0.025)*scaledamp*origSR/ma.SR;
                              combfeed = vslider("RoomSize[BELA: ANALOG_7]", 0.5, 0, 1, 0.001)*scaleroom*origSR/ma.SR + offsetroom;
                              spatSpread = vslider("Stereo",0.5,0,1,0.01)*46*ma.SR/origSR;
                              g = vslider("dryWetReverb[BELA: ANALOG_6]", 0.2, 0, 1, 0.001);
                              // (g = Dry/Wet)
                          };

                      // Dry-Wet (from C. LEBRETON)
                      dry_wetST(dw,x1,x2,y1,y2) = (wet*y1 + dry*x1),(wet*y2 + dry*x2)
                          with {
                              wet = 0.5*(dw+1.0);
                              dry = 1.0-wet;
                          };


)_";
  std::string error;
  m_factory = createDSPFactoryFromString("audioglitch", faustProgram, 0, nullptr, "", error);

  if(!m_factory)
    qDebug() << error.c_str();

  FaustUI ui{*this, *m_controls, m_guis};
  for(int i = 0; i < 4; i++) {
    m_dsp[i] = m_factory->createDSPInstance();
    m_dsp[i]->init(8000);

    ui.colorIndex = i;
    m_dsp[i]->buildUserInterface(&ui);
    ui.currentIndex = 0;
  }
}

MainWindow::~MainWindow()
{

}

void MainWindow::updateImage()
{
  QImage filteredImage = m_sourceImage;
  auto pixels = filteredImage.bits();
  std::size_t byteSize = filteredImage.bytesPerLine() * filteredImage.height();

  const int planes  = 4;
  if constexpr(1)
  {
    auto norm = [] (uchar c) -> float {
      return (c - 127) / 127.f;
    };
    auto denorm = [] (float c) -> uchar {
      return (c * 127.f) + 127;
    };

    std::vector<float> vecs[4];
    std::vector<float>& r = vecs[0], g = vecs[1], b = vecs[2], a = vecs[3];
    int numPixels = byteSize / planes;
    r.resize(numPixels);
    g.resize(numPixels);
    b.resize(numPixels);
    a.resize(numPixels);
    for(std::size_t i = 0; i < numPixels; i++)
    {
      r[i] = norm(pixels[planes * i]);
      g[i] = norm(pixels[planes * i+1]);
      b[i] = norm(pixels[planes * i+2]);
      a[i] = norm(pixels[planes * i+3]);
    }

    {
      float * ins[2] { r.data(), r.data() };
      float * outs[2] { r.data(), r.data() };
      m_dsp[0]->compute(r.size(), ins, outs);
    }

    {
      float * ins[2] { g.data(), g.data() };
      float * outs[2] { g.data(), g.data() };
      m_dsp[1]->compute(g.size(), ins, outs);
    }

    {
      float * ins[2] { b.data(), b.data() };
      float * outs[2] { b.data(), b.data() };
      m_dsp[2]->compute(b.size(), ins, outs);
    }

    for(std::size_t i = 0; i < numPixels; i++)
    {
      pixels[planes * i]   = denorm(r[i]); // blue
      pixels[planes * i+1] = denorm(g[i]); // green
      pixels[planes * i+2] = denorm(b[i]); // red
      pixels[planes * i+3] = denorm(a[i]); // alpha
    }
  }
  else if(0)
  {
    std::vector<float> values;
    values.resize(byteSize);
    for(std::size_t i = 0; i < byteSize; i++)
      values[i] = pixels[i];

    float * ins[2] { values.data(), values.data() };
    float * outs[2] { values.data(), values.data() };
    m_dsp[0]->compute(byteSize, ins, outs);

    for(std::size_t i = 0; i < byteSize; i++)
      pixels[i] = values[i];
  }
  else
  {
    float * ins[1] { reinterpret_cast<float*>(pixels) };
    float * outs[1] { reinterpret_cast<float*>(pixels) };
    m_dsp[0]->compute(byteSize / sizeof(float), ins, outs);
  }

  m_image->setPixmap(QPixmap::fromImage(filteredImage));

}

