import java.io.*;
import java.util.*;

import edu.cmu.sphinx.api.Configuration;
import edu.cmu.sphinx.api.SpeechResult;
import edu.cmu.sphinx.api.StreamSpeechRecognizer; 

//Usage: $java Experiments [ac_model] [dict] [lm] [results_dir] [recordings_path] [file_ids] [experiment name]

public class Experiments {
	public static void main (String[] args) throws Exception {
		Configuration configuration = new Configuration();
	
		//Sets parameters for recognition. 
		configuration.setAcousticModelPath(args[0]);
		configuration.setDictionaryPath(args[1]);
		configuration.setLanguageModelPath(args[2]); 

		//Paths for results and recordings. 
		String results_path = args[3];
		String recordings_path = args[4];
		String name = args[6];

		//Opens results files for writing. 
		String results_file_name = results_path + "/results.txt";
		FileOutputStream results_fos = new FileOutputStream(new File(results_file_name));
		BufferedWriter results_writer = new BufferedWriter(new OutputStreamWriter(results_fos));

		//Opens id file for reading. 
		FileReader fr = new FileReader(new File(args[5])); 
		BufferedReader reader = new BufferedReader(fr); 

		//Creates recognizer. 
		StreamSpeechRecognizer recognizer = new StreamSpeechRecognizer(configuration);
		
		String id = reader.readLine();

		while (id != null) {
			//Skips whitespace lines. 
			if (id.trim().length() > 0) {

				//Opens recording file. 
				String recording_name = recordings_path + "/" + id;
				System.out.println("FILE NAME: " + recording_name); 
				InputStream stream = new FileInputStream(new File(recording_name + ".raw"));

				//Recognizes utterance. 
				recognizer.startRecognition(stream);
				SpeechResult result = recognizer.getResult(); 
				recognizer.stopRecognition();

				//Writes recognition result. 
				String hyp = result.getHypothesis();
				results_writer.write(hyp + " (" + id + ")\n"); 

				//Writes top 1000 hypotheses for utterance. 
				Collection<String> nbest = result.getNbest(1000);
			
				//Opens file for writing hypotheses. 
				String nbest_file_name = results_path + "/" + id + ".nbest";
				FileOutputStream nbest_fos = new FileOutputStream(new File(nbest_file_name));
				BufferedWriter nbest_writer = new BufferedWriter(new OutputStreamWriter(nbest_fos));

				//Writes the hypotheses. 
				for (String hypothesis : nbest)
					nbest_writer.write(hypothesis + "\n");  

				id = reader.readLine(); 
			}
		}
	}
}
