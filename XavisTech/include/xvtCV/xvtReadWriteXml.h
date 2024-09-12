// Copyright (c) Xavis Tech, All rights are reserved.
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

// dependent headers
#include <xvtCV/xvtDefine.h>
#include <string>

namespace xvt {
		
typedef void* HXMLELEMENT;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ReadWriteXml is a class provided to easily handle XML objects.
// Get/set integer, real (double), string, and binary data in the Xml object using WriteXXXX / ReadXXXX methods.
// When creating an instance, you can use the E_XmlStorageTypes value to determine the binary data management method.
// The E_SimpleStorage type encodes binary data in base64 and manages the data inside xml.
// For the E_ComplexStorage type, data less than 4k is managed the same as the E_SimpleStorage type, and data more than 4k is managed as a separate file.
//
// The path parameter of the WriteXXXX / ReadXXXX method is a parameter to access a specific node in xml and is entered in xpath format.
// In the current version, even if there are multiple nodes with the same path, only the first node is accessed.
//
// [example]
// <data>
// <element1>
// <item1>value<item1>
// </element1>
// </data>
//
// To access item1, enter lpszPath = _T("/data/element1/item1").
// "." in the path parameter. If there is a character, it is replaced with "/".
// "/data/element1/item1.subitem1" == "/data/element1/item1/subitem1"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class XVT_EXPORTS ReadWriteXml
{
public:
	enum E_XmlStorageTypes
	{
		// Save all binaries to xml in base64 format.
		E_SimpleStorage = 0,

		// Only binaries less than 4K are stored in xml.
		// Data larger than 4k is saved as an external file and the path path is set in xml.
		E_ComplexStorage = 1,

		// Only binaries less than 4K are stored in xml.
		// Data larger than 4k is saved as an external file and the path path is set in xml.
		// When saving binary, compress and save.
		E_ComplexStorageWithCompression = 2,
	};

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// constructor and destructors
public:

	// constructor
	ReadWriteXml(E_XmlStorageTypes nType = E_ComplexStorage);

	// destructor
	virtual ~ReadWriteXml(void);


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// public methods
public:

	bool Load(std::wstring lpszXmlFilePath);
	bool Save(std::wstring lpszXmlFilePath);
	bool HasNode(std::wstring lpszPath) const;

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// write 

	bool WriteString(std::wstring lpszPath, std::wstring lpszValue);
	bool WriteInteger(std::wstring lpszPath, int nValue);
	bool WriteReal(std::wstring lpszPath, double dValue);
	bool WriteBool(std::wstring lpszPath, bool bValue);
	bool WriteBinary(std::wstring lpszPath, std::wstring &lpData, int dwDataLength);

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// read 
	std::wstring ReadString(std::wstring lpszPath, std::wstring lpszDefaultValue) const;
	int ReadInteger(std::wstring lpszPath, int nDefaultValue) const;
	double ReadReal(std::wstring lpszPath, double dDefaultValue) const;

	// Read data in binary form from storage.
	// If the data is large, it may take time.
	// If you know the size of the binary data you want to read, allocate memory externally and use it.
	// The input dwDataLength must be equal to or larger than the size of the actual stored data. If it is larger, the actual size is applied to dwDataLength after the function returns.
	// If lpData is entered as nullptr, the data is alloced internally.
	// Allocated memory must be deleted using the FreeBinary() method after use.
	bool ReadBinary(std::wstring lpszPath, std::wstring &lpData, int &dwDataLength) const;
	bool ReadBool(std::wstring lpszPath, bool bDefaultValue = false) const;
	static void FreeBinary(std::wstring lpData);

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// element handling functions

	// add element
	HXMLELEMENT AddElement(std::wstring lpszPath);
	HXMLELEMENT AddElement(HXMLELEMENT hElement, std::wstring lpszPath);

	// get element
	HXMLELEMENT GetElement(std::wstring lpszPath, int nIndex = 0) const;
	HXMLELEMENT GetElement(HXMLELEMENT hElement, std::wstring lpszElementName, int nIndex = 0) const;

	// get element count
	int GetElementCount(std::wstring lpszPath) const;
	int GetElementCount(HXMLELEMENT hElement, std::wstring lpszElementName) const;

	// remove element
	bool RemoveElement(std::wstring lpszPath, int nIndex);

	// remove item. 
	bool Remove(std::wstring lpszPath);
	std::wstring ReadString(HXMLELEMENT hElement, std::wstring lpszSubElementName, std::wstring lpszDefaultValue) const;
	int ReadInteger(HXMLELEMENT hElement, std::wstring lpszSubElementName, int nDefaultValue) const;
	bool ReadBool(HXMLELEMENT hElement, std::wstring lpszSubElementName, bool bValue) const;
	double ReadReal(HXMLELEMENT hElement, std::wstring lpszSubElementName, double dDefaultValue) const;
	bool ReadBinary(HXMLELEMENT hElement, std::wstring lpszSubElementName, std::wstring &lpData) const;
	bool WriteString(HXMLELEMENT hElement, std::wstring lpszSubElementName, std::wstring lpszValue);
	bool WriteInteger(HXMLELEMENT hElement, std::wstring lpszSubElementName, int nValue);
	bool WriteReal(HXMLELEMENT hElement, std::wstring lpszSubElementName, double dValue);
	bool WriteBool(HXMLELEMENT hElement, std::wstring lpszSubElementName, bool bValue);
	bool WriteBinary(HXMLELEMENT hElement, std::wstring lpszSubElementName, std::wstring& lpData, int dwDataLength);

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// attribute handling functions

	// get attribute
	std::wstring GetAttribute(HXMLELEMENT hElement, std::wstring lpszAttributeName, std::wstring lpszDefaultValue) const;

	// set attribute
	bool SetAttribute(HXMLELEMENT hElement, std::wstring lpszAttributeName, std::wstring lpszValue);

	// remove attribute
	bool RemoveAttribute(HXMLELEMENT hElement, std::wstring lpszAttributeName);


	// read attibute string
	std::wstring ReadAttributeString(HXMLELEMENT hElement, std::wstring lpszAttributeName, std::wstring lpszDefaultValue) const;

	int ReadAttributeInteger(HXMLELEMENT hElement, std::wstring lpszSubElementName, int nDefaultValue) const;

	double ReadAttributeReal(HXMLELEMENT hElement, std::wstring lpszSubElementName, double dDefaultValue) const;

	float ReadAttributeFloat(HXMLELEMENT hElement, std::wstring lpszSubElementName, float fDefaultValue) const;


	// write attibute string
	bool WriteAttributeString(HXMLELEMENT hElement, std::wstring lpszAttributeName, std::wstring lpszValue);

	bool WriteAttributeInteger(HXMLELEMENT hElement, std::wstring lpszSubElementName, int nValue);

	bool WriteAttributeBool(HXMLELEMENT hElement, std::wstring lpszSubElementName, bool bValue);

	bool WriteAttributeReal(HXMLELEMENT hElement, std::wstring lpszSubElementName, double dValue, bool bNoTrailZero = false);

	bool WriteAttributeFloat(HXMLELEMENT hElement, std::wstring lpszSubElementName, float fValue, bool bNoTrailZero = false);


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// protected methods
protected:

	bool Initialize(void);
	bool CleanUp(void);
	bool GetString(std::wstring lpszPath, std::wstring* pstrValue) const;
	const std::wstring& GetCurrentXmlFilePath(void) const;
	void* GetXmlRoot(void) const { return _pCurrentXml; }
	E_XmlStorageTypes GetXmlBinaryStoreType(void) const { return _nType; }
	bool InitializeTempDirectory(void);
	bool GetBinary(std::wstring lpszHint, std::wstring &lpData, int dwDataLength) const;
	bool SetExternalBinary(const std::wstring lpszHint, const std::wstring lpData, int dwDataLength, bool bCompressed, std::wstring* pstrFileName);


	// find element
	HXMLELEMENT FindElement(HXMLELEMENT hElement, std::wstring lpszSubElementName, bool bCreate = false) const;

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// privated data
private:

	void*				_pCurrentXml;
	std::wstring		_strCurrentXmlFilePath;
	std::wstring		_strTempDirectory;
	E_XmlStorageTypes	_nType;

	std::wstring		_strComment;

};

		
}  // namespace xvt



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// for convenience. 
#define XVT_XmlParser				::xvt::ReadWriteXml
#define XVT_HXMLELEMENT				::xvt::HXMLELEMENT

