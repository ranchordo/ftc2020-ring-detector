package main;

import java.awt.BorderLayout;
import java.awt.Graphics;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.io.File;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.SwingWorker;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;

public class Main extends JPanel {
	private static final long serialVersionUID = -628559266691870563L;
	public static void addData(String a, String b) {
		System.out.println(a+": "+b);
	}
	BufferedImage image;
	public void updateImage(Mat matrix) {
		int type = BufferedImage.TYPE_BYTE_GRAY;
		if (matrix.channels() > 1) {
			type = BufferedImage.TYPE_3BYTE_BGR;
		}
		byte[] b = new byte[matrix.channels() * matrix.cols() * matrix.rows()];
		matrix.get(0, 0, b);
		image = new BufferedImage(matrix.cols(), matrix.rows(), type);
		final byte[] targetPixels = ((DataBufferByte) image.getRaster().getDataBuffer()).getData();
		System.arraycopy(b, 0, targetPixels, 0, b.length);

		repaint();
	}
	public static boolean closed=false;
	@Override
	public void paintComponent(Graphics g) {
		super.paintComponent(g);
		if (image == null) {
			return;
		}
		//g.setColor(new Color(255,0,255));
		//g.drawRect(50,50,100,100);
		g.drawImage(this.image, 1, 1, this.image.getWidth(), this.image.getHeight(), null);
	}
	public static String getExternalPath() {
		File jarpath=new File(Main.class.getProtectionDomain().getCodeSource().getLocation().getPath());
		String externalPath=jarpath.getParentFile().getAbsolutePath().replace("\\", "/").replace("%20", " ");
		return externalPath;
	}
	public static void main(String[] args) {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		VideoCapture camera = new VideoCapture(0,Videoio.CAP_DSHOW);
		if(!camera.isOpened()){
			System.out.println("Error opening camera");
			System.exit(1);
		}
		System.out.println("VideoCapture inited.");
		//System.exit(1);
		Main panel = new Main();
		JFrame app = new JFrame("Ring detection");
		app.add(panel, BorderLayout.CENTER);
		app.addWindowListener(new WindowAdapter() {
            @Override
            public void windowClosing(WindowEvent e) {
                closed=true;
                System.exit(0);
            }
        });
		app.setSize(640, 480);
		app.setLocationRelativeTo(null);
		app.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		SwingWorker<Void, Mat> worker = new SwingWorker<Void, Mat>() {
			@Override
			protected Void doInBackground() throws Exception {
				Mat webcamImage = new Mat();
				while (!isCancelled()) {
					if(closed) {
						System.out.println("CLOSE");
						this.cancel(true);
						
					}
					camera.read(webcamImage);
					Mat processed=null;
					if (!webcamImage.empty()) {
						try {
						processed=Processor.process(webcamImage);
						} catch (Exception e) {
							e.printStackTrace();
						}
						try {
							panel.updateImage(processed);
						} catch (Exception e) {
							
						}
					} else {
						break;
					}
				}

				return null;
			}
		};
		worker.execute();
		app.setVisible(true);
	}
}