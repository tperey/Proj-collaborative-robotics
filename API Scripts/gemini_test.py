import google.generativeai as genai

genai.configure(api_key='AIzaSyCptM2l3SVH6ZBbsST_2m_haTIG-pCslyc')
model = genai.GenerativeModel("gemini-1.5-flash")
response = model.generate_content("Explain how AI works")
print(response.text)