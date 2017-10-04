from tl_detector.light_classification.tl_classifier import *

def main():
	clsf = TLClassifier()
	predict = clsf.get_classification('test_green1.jpg')
	print (predict)

if __name__ == '__main__':
	main()